# clock_stepper.py
# Python 3.x — Raspberry Pi OS
#
# Dependencies (install as needed):
#   pip install RPi.GPIO pyserial
#   (Your TCM-2209-Raspberry-Pi module — adjust the import below)
#
# Wiring assumptions (change to match your setup):
#   - TMC2209 STEP pin -> STEP_PIN
#   - TMC2209 DIR  pin -> DIR_PIN
#   - TMC2209 EN   pin -> ENABLE_PIN (low = enabled on many boards)
#   - Minute-hand home switch -> MINUTE_HOME_PIN (pull-up, active-low)
#   - Hour-hand   home switch -> HOUR_HOME_PIN   (pull-up, active-low)
#
# Homing logic:
#   - Slowly step forward until BOTH home switches read active at the same time.
#   - Declare that mechanical alignment as "midnight" (absolute position = 0 steps).
#   - Safety: limit the total steps taken during homing.
#
# Movement:
#   - goto_minutes(minutes_since_midnight) converts minutes->absolute steps
#     and moves shortest signed distance (or always forward if you prefer).
#   - Steps are cast to int for the move request per your requirement.

import time
import threading
import math

import RPi.GPIO as GPIO

# Try a few plausible import names for the TMC2209 module.
# Replace with the exact module / class you use if needed.
_tmc2209_cls = None
_tmc_import_error = None
try:
    from TMC2209 import TMC2209  # common pypi module name
    _tmc2209_cls = TMC2209
except Exception as e1:
    try:
        from tmc2209 import TMC2209  # alternate
        _tmc2209_cls = TMC2209
    except Exception as e2:
        _tmc_import_error = (e1, e2)
        # We proceed anyway so you can still test GPIO/homing logic.
        # You can wire the class to your exact driver API in _configure_driver().


class ClockStepperController:
    """
    Controls a single stepper (via TMC2209) that drives both minute and hour hands.
    Provides:
      - home(): find the simultaneous home of both hands
      - goto_minutes(minutes_since_midnight: float): move motor to absolute time position
    """

    def __init__(
        self,
        *,
        serial_port="/dev/serial0",
        uart_addr=0,                 # TMC2209 slave address if your lib uses it
        step_pin=5,                 # BCM numbering
        dir_pin=6,
        enable_pin=4,
        minute_home_pin=26,          # minute-hand switch input (BCM)
        hour_home_pin=24,            # hour-hand switch input (BCM)
        steps_per_rev=200,           # full steps per motor rev
        minute_per_motor_rev=10/3,   # minute hand revs per motor rev (your spec)
        hour_per_minute_rev=1/18,    # hour hand revs per minute-hand rev (your spec)
        microsteps=8,                # set microstepping in driver if you like
        run_speed_sps=800,           # default run speed: steps per second
        home_speed_sps=200,          # slow homing speed: steps per second
        debounce_ms=8,               # switch debounce
        enable_low_is_on=True,       # most TMC carrier boards: EN low = enable
        invert_dir=False             # flip if motor goes the wrong way
    ):
        self.serial_port = serial_port
        self.uart_addr = uart_addr
        self.STEP_PIN = step_pin
        self.DIR_PIN = dir_pin
        self.ENABLE_PIN = enable_pin
        self.MINUTE_HOME_PIN = minute_home_pin
        self.HOUR_HOME_PIN = hour_home_pin

        self.steps_per_rev = int(steps_per_rev)
        self.minute_per_motor_rev = float(minute_per_motor_rev)
        self.hour_per_minute_rev = float(hour_per_minute_rev)
        self.microsteps = int(microsteps)
        self.run_speed_sps = float(run_speed_sps)
        self.home_speed_sps = float(home_speed_sps)
        self.debounce_s = debounce_ms / 1000.0
        self.enable_low_is_on = bool(enable_low_is_on)
        self.invert_dir = bool(invert_dir)

        # Internal state
        self._abs_pos_steps = 0  # absolute position (full-step units) relative to "midnight"
        self._pos_lock = threading.Lock()

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.STEP_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.DIR_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.ENABLE_PIN, GPIO.OUT, initial=GPIO.HIGH if self.enable_low_is_on else GPIO.LOW)

        # Pull-ups for home switches; active-low assumed (change if needed)
        GPIO.setup(self.MINUTE_HOME_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.HOUR_HOME_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Debounce helpers
        self._last_minute_read_time = 0.0
        self._last_minute_state = self._read_raw_minute()
        self._last_hour_read_time = 0.0
        self._last_hour_state = self._read_raw_hour()

        # Driver object (configure via UART if available)
        self.driver = None
        self._configure_driver()

    # ---------------------------
    # Hardware helpers
    # ---------------------------

    def _configure_driver(self):
        """Configure the TMC2209 over UART (if the module is available)."""
        if _tmc2209_cls is None:
            # Module import failed — still allow stepping via STEP/DIR.
            # You can wire your specific driver class here if different.
            return

        # Example: many TMC2209 libs accept serial_port and/or addr.
        try:
            # The constructor signature varies across libraries;
            # you may need to adjust these lines for your module.
            self.driver = _tmc2209_cls(serial_port=self.serial_port, addr=self.uart_addr)
            # Typical configuration (adjust to taste / module API):
            # Current and microstepping configuration may differ per lib.
            try:
                # Common patterns — comment/uncomment based on your module:
                # self.driver.set_microstepping(self.microsteps)
                # self.driver.set_irun(20)      # run current %
                # self.driver.set_ihold(8)      # hold current %
                # self.driver.enable_spreadcycle(False)  # stealthChop on
                pass
            except Exception:
                pass
        except Exception:
            # If driver init fails, you can still step using STEP/DIR
            self.driver = None

    def _enable_motor(self, enable=True):
        GPIO.output(
            self.ENABLE_PIN,
            GPIO.LOW if (enable and self.enable_low_is_on) else
            GPIO.HIGH if (enable and not self.enable_low_is_on) else
            (GPIO.HIGH if self.enable_low_is_on else GPIO.LOW)
        )

    def _set_dir(self, forward=True):
        level = GPIO.LOW if (forward ^ self.invert_dir) else GPIO.HIGH
        GPIO.output(self.DIR_PIN, level)

    def _pulse_step(self, pulse_us=4):
        # Keep STEP low-high-low — many drivers accept very short pulses; 4-10 µs is safe.
        GPIO.output(self.STEP_PIN, GPIO.HIGH)
        time.sleep(pulse_us / 1_000_000.0)
        GPIO.output(self.STEP_PIN, GPIO.LOW)
        # Short hold-low; the inter-step delay is handled by outer timing.

    def _read_raw_minute(self):
        # Returns True when switch is ACTIVE.
        # If your switch is active-high, invert this.
        return GPIO.input(self.MINUTE_HOME_PIN) == GPIO.LOW

    def _read_raw_hour(self):
        return GPIO.input(self.HOUR_HOME_PIN) == GPIO.LOW

    def _debounced_minute_active(self):
        now = time.time()
        state = self._read_raw_minute()
        if state != self._last_minute_state:
            # changed — accept only if stable for debounce period
            if (now - self._last_minute_read_time) >= self.debounce_s:
                self._last_minute_state = state
                self._last_minute_read_time = now
        return self._last_minute_state

    def _debounced_hour_active(self):
        now = time.time()
        state = self._read_raw_hour()
        if state != self._last_hour_state:
            if (now - self._last_hour_read_time) >= self.debounce_s:
                self._last_hour_state = state
                self._last_hour_read_time = now
        return self._last_hour_state

    # ---------------------------
    # Kinematics (your ratios)
    # ---------------------------

    def _minutes_to_abs_steps(self, minutes_since_midnight: float) -> int:
        """
        Map minutes -> absolute motor steps using your ratios.

        minute_hand_revs = minutes / 60
        motor_revs = minute_hand_revs / (minute_per_motor_rev)
                   = (minutes / 60) * (3/10)   [since minute_per_motor_rev = 10/3]
                   = minutes / 200
        steps = steps_per_rev * motor_revs
              = steps_per_rev * (minutes / 200)

        With steps_per_rev = 200 => steps = minutes.
        We cast to int (truncate) as requested.
        """
        steps_float = self.steps_per_rev * (minutes_since_midnight / 200.0)
        return int(steps_float)

    # ---------------------------
    # Public API
    # ---------------------------

    def home(self, max_search_steps=4000):
        """
        Rotate slowly forward until BOTH home switches are active at the same time.
        Declare that alignment as absolute position 0 ("midnight").
        Safety stops after max_search_steps if not found.
        """
        self._enable_motor(True)
        self._set_dir(True)  # forward

        step_interval = 1.0 / max(self.home_speed_sps, 1.0)
        steps_moved = 0

        # Clear debounce baseline
        _ = self._debounced_minute_active()
        _ = self._debounced_hour_active()
        time.sleep(self.debounce_s)

        try:
            while steps_moved < max_search_steps:
                # step one
                self._pulse_step()
                time.sleep(step_interval)

                steps_moved += 1

                minute_on = self._debounced_minute_active()
                hour_on = self._debounced_hour_active()

                if minute_on and hour_on:
                    # Found simultaneous home
                    with self._pos_lock:
                        self._abs_pos_steps = 0
                    self._enable_motor(False)
                    return True

            # If we got here, not found
            self._enable_motor(False)
            return False

        except KeyboardInterrupt:
            self._enable_motor(False)
            return False

    def goto_minutes(self, minutes_since_midnight: float, speed_sps=None):
        """
        Move to the absolute position corresponding to minutes_since_midnight.
        Steps to move are cast to int (truncate).
        """
        if speed_sps is None:
            speed_sps = self.run_speed_sps

        target_abs = self._minutes_to_abs_steps(float(minutes_since_midnight))
        with self._pos_lock:
            delta = target_abs - self._abs_pos_steps

        # Choose direction and magnitude
        forward = delta >= 0
        steps_to_move = abs(int(delta))  # ensure int

        if steps_to_move == 0:
            return 0

        self._enable_motor(True)
        self._set_dir(forward)

        step_interval = 1.0 / max(speed_sps, 1.0)

        try:
            for _ in range(steps_to_move):
                self._pulse_step()
                time.sleep(step_interval)
            # Update absolute position
            with self._pos_lock:
                self._abs_pos_steps += (steps_to_move if forward else -steps_to_move)
            self._enable_motor(False)
            return steps_to_move
        except KeyboardInterrupt:
            self._enable_motor(False)
            return 0

    def get_position_minutes(self) -> float:
        """
        Report the current absolute position in minutes (float), derived
        from the internal absolute step counter and the ratio.
        """
        with self._pos_lock:
            steps = self._abs_pos_steps
        # Invert steps->minutes: steps = steps_per_rev * (minutes / 200)
        # => minutes = 200 * steps / steps_per_rev
        return (200.0 * steps) / float(self.steps_per_rev)

    def cleanup(self):
        """Release GPIO (call on program exit)."""
        try:
            self._enable_motor(False)
        finally:
            GPIO.cleanup()

if __name__ == "__main__":
    clock = ClockStepperController(
        serial_port="/dev/serial0",
        uart_addr=0,              # adjust if your board uses an address
        step_pin=5,
        dir_pin=6,
        enable_pin=4,
        minute_home_pin=26,
        hour_home_pin=24,
        steps_per_rev=200,        # your motor
        minute_per_motor_rev=10/3,
        hour_per_minute_rev=1/18,
        microsteps=8,             # optional, if configured in your driver
        run_speed_sps=800,
        home_speed_sps=200,
        debounce_ms=8,
        enable_low_is_on=True,
        invert_dir=False
    )

    # 1) Home both hands (stop when both switches are active at once)
    #ok = clock.home()
    #print("Homed:", ok)

    # 2) Move to 07:30:00 AM → 7.5 hours → 450.0 minutes
    moved = clock.goto_minutes(450.0)
    print("Steps moved:", moved)
    print("Now at minutes = ", clock.get_position_minutes())

    # 3) Move to 23:59 (1439 minutes)
    clock.goto_minutes(1439.0)

    # Cleanup on exit
    clock.cleanup()
