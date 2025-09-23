# motor_controller.py
# Raspberry Pi Zero 2 W — TMC2209 stepper clock controller with soft idle / auto-disable
#
# Dependencies:
#   pip install RPi.GPIO pyserial
#   (Install your TCM-2209-Raspberry-Pi library; update imports below if names differ)
#
# Features:
#   - STEP/DIR timing safety (DIR setup delay, wider STEP pulse, enable settle)
#   - Gentle trapezoid accel/decel
#   - Homing until BOTH switches are active simultaneously
#   - Minutes→motor-steps math using your gear ratios (200 steps/rev ⇒ steps = minutes)
#   - Explicit run/hold current writes via high-level API or raw UART fallback to IHOLD_IRUN (0x10)
#   - spreadCycle optional (for reliable low-speed torque)
#   - Soft idle: keep EN enabled but drop IHOLD to a small value, OR fully disable outputs after inactivity
#   - Auto-disable timer (configurable via auto_disable_ms)

import time
import threading
import RPi.GPIO as GPIO

# ---- TMC2209 library import (best-effort, adapt if your package name differs) ----
_tmc2209_cls = None
try:
    from TMC2209 import TMC2209
    _tmc2209_cls = TMC2209
except Exception:
    try:
        from tmc2209 import TMC2209
        _tmc2209_cls = TMC2209
    except Exception:
        _tmc2209_cls = None

# ---- TMC2209 register constants (for raw UART fallback) ----
IHOLD_IRUN_ADDR = 0x10  # Bits: 0..4 IHOLD, 8..12 IRUN, 16..19 IHOLDDELAY


def _u32(x):  # ensure 32-bit unsigned
    return x & 0xFFFFFFFF


class ClockStepperController:
    """
    Controls a single stepper (via TMC2209 STEP/DIR) driving both minute & hour hands.
    Provides:
      - home(): rotate until BOTH home switches active simultaneously, set abs pos = 0
      - goto_minutes(minutes_since_midnight: float): seek absolute time position
      - get_position_minutes(): report current absolute minutes (float)
      - motor_on() / motor_off() with auto-disable and soft-idle support
      - cleanup(): release GPIO

    Ratios (from your spec):
      minute hand revs per motor rev  = 10/3
      hour hand   revs per minute rev = 1/18

    From this:
      minute_hand_revs  = minutes / 60
      motor_revs        = minute_hand_revs / (10/3) = (minutes / 60) * (3/10) = minutes / 200
      steps             = steps_per_rev * motor_revs = steps_per_rev * (minutes / 200)

    With steps_per_rev = 200  =>  steps = minutes (we keep the general formula).
    """

    def __init__(
        self,
        *,
        # --- UART / driver ---
        serial_port="/dev/serial0",
        uart_addr=0,

        # --- STEP/DIR/EN pins (BCM numbering) ---
        step_pin=5,
        dir_pin=6,
        enable_pin=4,

        # --- Home switches (BCM) - assumed active-low with pull-ups ---
        minute_home_pin=26,
        hour_home_pin=24,

        # --- Mechanics ---
        steps_per_rev=200,
        minute_per_motor_rev=10/3,
        hour_per_minute_rev=1/18,

        # --- Motion ---
        microsteps=16,
        run_speed_sps=800,    # cruise speed (steps/sec)
        home_speed_sps=120,   # slower for reliable homing
        accel_sps2=2000,      # accel/decel (steps/sec^2)

        # --- Electrical / timings ---
        debounce_ms=8,
        enable_low_is_on=True,  # many carriers: EN low = enabled
        invert_dir=False,       # flip if direction is reversed
        step_pulse_us=10,       # STEP high time
        step_low_us=10,         # extra low time to enforce min period
        dir_setup_us=60,        # wait after DIR change before stepping
        enable_settle_ms=5,     # wait after enabling driver before step

        # --- Driver config ---
        tmc_enable_spreadcycle=True,  # disable stealthChop for stronger low-speed torque
        irun_percent=35,              # run current (% of full-scale code)
        ihold_percent=15,             # hold current (%) while active
        ihold_delay=4,                # 0..15 (smoother run->hold transition)

        # --- Idle behavior ---
        auto_disable_ms=3000,         # disable after inactivity (0 = never)
        soft_idle=False,              # if True, keep EN enabled at idle, but lower IHOLD
        idle_ihold_percent=5          # IHOLD used during soft idle
    ):
        # Save config
        self.serial_port = serial_port
        self.uart_addr = uart_addr

        self.STEP_PIN = step_pin
        self.DIR_PIN = dir_pin
        self.ENABLE_PIN = enable_pin
        self.MINUTE_HOME_PIN = minute_home_pin
        self.HOUR_HOME_PIN = hour_home_pin

        # Pin conflict guard
        pins = {
            "STEP": self.STEP_PIN,
            "DIR": self.DIR_PIN,
            "EN": self.ENABLE_PIN,
            "MIN_HOME": self.MINUTE_HOME_PIN,
            "HOUR_HOME": self.HOUR_HOME_PIN,
        }
        rev = {}
        for name, p in pins.items():
            if p in rev:
                raise ValueError(f"Pin conflict: {name} and {rev[p]} both set to BCM {p}")
            rev[p] = name

        self.steps_per_rev = int(steps_per_rev)
        self.minute_per_motor_rev = float(minute_per_motor_rev)
        self.hour_per_minute_rev = float(hour_per_minute_rev)

        self.microsteps = int(microsteps)
        self.run_speed_sps = float(run_speed_sps)
        self.home_speed_sps = float(home_speed_sps)
        self.accel_sps2 = float(accel_sps2)

        self.debounce_s = debounce_ms / 1000.0
        self.enable_low_is_on = bool(enable_low_is_on)
        self.invert_dir = bool(invert_dir)

        self.step_pulse_us = int(step_pulse_us)
        self.step_low_us = int(step_low_us)
        self.dir_setup_us = int(dir_setup_us)
        self.enable_settle_ms = int(enable_settle_ms)

        self.tmc_enable_spreadcycle = bool(tmc_enable_spreadcycle)
        self.irun_percent = int(irun_percent)
        self.ihold_percent = int(ihold_percent)
        self.ihold_delay = int(ihold_delay)

        # Idle behavior
        self.auto_disable_ms = int(auto_disable_ms)
        self.soft_idle = bool(soft_idle)
        self.idle_ihold_percent = int(idle_ihold_percent)
        self._idle_timer = None

        # Internal absolute position (full-step units) relative to "midnight"
        self._abs_pos_steps = 0
        self._pos_lock = threading.Lock()

        # GPIO init
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.STEP_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.DIR_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.ENABLE_PIN, GPIO.OUT,
                   initial=GPIO.HIGH if self.enable_low_is_on else GPIO.LOW)
        GPIO.setup(self.MINUTE_HOME_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.HOUR_HOME_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Debounce baselines
        self._last_minute_state = (GPIO.input(self.MINUTE_HOME_PIN) == GPIO.LOW)
        self._last_hour_state = (GPIO.input(self.HOUR_HOME_PIN) == GPIO.LOW)
        self._last_minute_time = time.time()
        self._last_hour_time = time.time()

        # TMC2209 driver object
        self.driver = None
        self._configure_driver()

    # ----------------------------------------------------------------------------
    # Driver configuration (with explicit hold/run current writes)
    # ----------------------------------------------------------------------------

    def _configure_driver(self):
        """Create the TMC2209 object and apply microsteps, currents, and mode."""
        if _tmc2209_cls is not None:
            try:
                self.driver = _tmc2209_cls(serial_port=self.serial_port, addr=self.uart_addr)
            except Exception:
                self.driver = None

        # Microstepping (best-effort across common method names)
        if self.driver is not None:
            for name in ("set_microstepping", "setMicrostepping", "microsteps", "set_microsteps"):
                try:
                    getattr(self.driver, name)(self.microsteps)
                    break
                except Exception:
                    pass

            # Mode: prefer spreadCycle for homing/low-speed torque
            if self.tmc_enable_spreadcycle:
                for name, args in (
                    ("enable_spreadcycle", (True,)),
                    ("enableSpreadCycle", (True,)),
                    ("enable_stealth_chop", (False,)),
                    ("enableStealthChop", (False,)),
                    ("set_spreadcycle", (True,)),
                ):
                    try:
                        getattr(self.driver, name)(*args)
                    except Exception:
                        pass

            # Apply run/hold currents (percent) — try high-level APIs, then raw reg write
            self._apply_currents_percent(
                irun_percent=self.irun_percent,
                ihold_percent=self.ihold_percent,
                ihold_delay=self.ihold_delay
            )

    def _apply_currents_percent(self, irun_percent: int, ihold_percent: int, ihold_delay: int = 4):
        """Set run/hold current. Falls back to direct IHOLD_IRUN write if needed."""
        if self.driver is None:
            return

        # 1) Try common high-level percent setters
        tried_any = False
        for name, is_run in (
            ("set_run_current", True),
            ("setRunCurrent", True),
            ("set_run_current_percent", True),
            ("set_hold_current", False),
            ("setHoldCurrent", False),
            ("set_hold_current_percent", False),
        ):
            try:
                if is_run:
                    getattr(self.driver, name)(irun_percent)
                else:
                    getattr(self.driver, name)(ihold_percent)
                tried_any = True
            except Exception:
                pass

        # 2) Combined setter if present
        if not tried_any:
            for name in ("set_current", "setCurrent", "set_currents"):
                try:
                    getattr(self.driver, name)(irun_percent, ihold_percent)
                    tried_any = True
                    break
                except Exception:
                    pass

        # 3) Raw register write fallback
        if not tried_any:
            self._raw_write_ihold_irun(irun_percent, ihold_percent, ihold_delay)

    def _raw_write_ihold_irun(self, irun_percent: int, ihold_percent: int, ihold_delay: int):
        """
        Write IHOLD_IRUN (0x10) directly:
          bits  0..4  IHOLD (0..31)
          bits  8..12 IRUN  (0..31)
          bits 16..19 IHOLDDELAY (0..15)
        """
        if self.driver is None:
            return

        # Map percent (0..100) to 5-bit code (0..31)
        to_code = lambda p: max(0, min(31, round((p / 100.0) * 31.0)))
        ihold_code = int(to_code(ihold_percent))
        irun_code  = int(to_code(irun_percent))
        ihdly_code = max(0, min(15, int(ihold_delay)))

        value = (ihold_code & 0x1F) | ((irun_code & 0x1F) << 8) | ((ihdly_code & 0x0F) << 16)
        value = _u32(value)

        # Try a few common low-level write method names
        for name, args in (
            ("write_register", (IHOLD_IRUN_ADDR, value)),
            ("writeRegister", (IHOLD_IRUN_ADDR, value)),
            ("write_reg", (IHOLD_IRUN_ADDR, value)),
            ("writeReg", (IHOLD_IRUN_ADDR, value)),
            ("reg_write", (IHOLD_IRUN_ADDR, value)),
            ("register_write", (IHOLD_IRUN_ADDR, value)),
        ):
            try:
                getattr(self.driver, name)(*args)  # UART bytes go out inside this call
                return
            except Exception:
                pass
        # If none exist, this driver lib doesn't expose raw writes.

    # ----------------------------------------------------------------------------
    # Idle helpers (soft idle / auto-disable)
    # ----------------------------------------------------------------------------

    def motor_on(self):
        """Enable driver output stage (holding or run current active)."""
        self._enable_motor(True)
        # If soft idle adjusted IHOLD previously, restore normal IHOLD now
        if self.soft_idle and self.driver is not None:
            try:
                self._apply_currents_percent(self.irun_percent, self.ihold_percent, self.ihold_delay)
            except Exception:
                pass
        # cancel any pending auto-disable
        if self._idle_timer:
            try:
                self._idle_timer.cancel()
            except Exception:
                pass
            self._idle_timer = None

    def motor_off(self):
        """Disable driver output stage (no holding current)."""
        self._enable_motor(False)
        if self._idle_timer:
            try:
                self._idle_timer.cancel()
            except Exception:
                pass
            self._idle_timer = None

    def _schedule_idle_action(self):
        """After inactivity, either fully disable EN or apply soft idle IHOLD."""
        if self.auto_disable_ms <= 0:
            return
        # cancel existing timer
        if self._idle_timer:
            try:
                self._idle_timer.cancel()
            except Exception:
                pass
        def _idle_action():
            if self.soft_idle:
                # Keep EN enabled, reduce IHOLD
                if self.driver is not None:
                    try:
                        self._apply_currents_percent(self.irun_percent, self.idle_ihold_percent, self.ihold_delay)
                    except Exception:
                        pass
            else:
                # Fully disable outputs
                self.motor_off()
        t = threading.Timer(self.auto_disable_ms / 1000.0, _idle_action)
        t.daemon = True
        t.start()
        self._idle_timer = t

    # ----------------------------------------------------------------------------
    # GPIO / stepping helpers
    # ----------------------------------------------------------------------------

    def _enable_motor(self, enable=True):
        GPIO.output(
            self.ENABLE_PIN,
            GPIO.LOW if (enable and self.enable_low_is_on) else
            GPIO.HIGH if (enable and not self.enable_low_is_on) else
            (GPIO.HIGH if self.enable_low_is_on else GPIO.LOW)
        )
        if enable:
            time.sleep(self.enable_settle_ms / 1000.0)

    def _set_dir(self, forward=True):
        level = GPIO.LOW if (forward ^ self.invert_dir) else GPIO.HIGH
        GPIO.output(self.DIR_PIN, level)
        time.sleep(self.dir_setup_us / 1_000_000.0)  # DIR setup time

    def _pulse_step(self):
        GPIO.output(self.STEP_PIN, GPIO.HIGH)
        time.sleep(self.step_pulse_us / 1_000_000.0)
        GPIO.output(self.STEP_PIN, GPIO.LOW)
        time.sleep(self.step_low_us / 1_000_000.0)

    # Debounced home switches (assumed active-low)
    def _minute_active(self):
        now = time.time()
        state = (GPIO.input(self.MINUTE_HOME_PIN) == GPIO.LOW)
        if state != self._last_minute_state and (now - self._last_minute_time) >= self.debounce_s:
            self._last_minute_state = state
            self._last_minute_time = now
        return self._last_minute_state

    def _hour_active(self):
        now = time.time()
        state = (GPIO.input(self.HOUR_HOME_PIN) == GPIO.LOW)
        if state != self._last_hour_state and (now - self._last_hour_time) >= self.debounce_s:
            self._last_hour_state = state
            self._last_hour_time = now
        return self._last_hour_state

    # ----------------------------------------------------------------------------
    # Mapping: minutes <-> steps
    # ----------------------------------------------------------------------------

    def _minutes_to_abs_steps(self, minutes_since_midnight: float) -> int:
        # motor_revs = minutes / 200; steps = steps_per_rev * motor_revs
        steps_float = self.steps_per_rev * (float(minutes_since_midnight) / 200.0)
        return int(steps_float)  # cast to int exactly as requested

    def get_position_minutes(self) -> float:
        with self._pos_lock:
            steps = self._abs_pos_steps
        return (200.0 * steps) / float(self.steps_per_rev)

    # ----------------------------------------------------------------------------
    # Motion primitive with simple trapezoid acceleration
    # ----------------------------------------------------------------------------

    def _move_steps_trapezoid(self, steps: int, vmax_sps: float):
        """Move signed 'steps' with accel/decel for smooth start/stop."""
        if steps == 0:
            # Even if no motion, schedule idle action
            self._schedule_idle_action()
            return
        forward = steps > 0
        n = abs(int(steps))

        self.motor_on()
        self._set_dir(forward)

        # Basic trapezoid: s_accel = v^2 / (2a) (limit to half distance)
        a = max(self.accel_sps2, 1.0)
        vmax = max(vmax_sps, 50.0)
        s_accel = int(max(1, round((vmax * vmax) / (2.0 * a))))
        s_accel = min(s_accel, n // 2)
        s_cruise = n - 2 * s_accel

        # Accel ramp (start at a gentle speed)
        v = 50.0
        for _ in range(s_accel):
            v = min(vmax, v + a / max(v, 1.0))  # simple ramp
            dt = 1.0 / v
            self._pulse_step()
            remaining = max(0.0, dt - (self.step_pulse_us + self.step_low_us)/1_000_000.0)
            if remaining > 0:
                time.sleep(remaining)

        # Cruise
        if s_cruise > 0:
            dt = 1.0 / vmax
            base_sleep = max(0.0, dt - (self.step_pulse_us + self.step_low_us)/1_000_000.0)
            for _ in range(s_cruise):
                self._pulse_step()
                if base_sleep > 0:
                    time.sleep(base_sleep)

        # Decel (mirror accel)
        for i in range(s_accel, 0, -1):
            v_est = max(50.0, (i * a) ** 0.5)
            v = min(v_est, vmax)
            dt = 1.0 / v
            self._pulse_step()
            remaining = max(0.0, dt - (self.step_pulse_us + self.step_low_us)/1_000_000.0)
            if remaining > 0:
                time.sleep(remaining)

        # After motion completes, schedule idle action (soft idle or full disable)
        self._schedule_idle_action()

    # ----------------------------------------------------------------------------
    # Public API
    # ----------------------------------------------------------------------------

    def home(self, max_search_steps=4000):
        """
        Rotate forward at homing speed until BOTH switches are active simultaneously.
        Set absolute position to 0 (midnight). Returns True on success, False if not found.
        """
        self.motor_on()
        self._set_dir(True)  # forward

        v = max(self.home_speed_sps, 60.0)
        dt = 1.0 / v
        base_sleep = max(0.0, dt - (self.step_pulse_us + self.step_low_us)/1_000_000.0)

        # Settle debouncers
        _ = self._minute_active()
        _ = self._hour_active()
        time.sleep(self.debounce_s)

        steps_moved = 0
        try:
            while steps_moved < max_search_steps:
                self._pulse_step()
                steps_moved += 1
                if base_sleep > 0:
                    time.sleep(base_sleep)

                if self._minute_active() and self._hour_active():
                    with self._pos_lock:
                        self._abs_pos_steps = 0
                    # Idle behavior after successful homing
                    self._schedule_idle_action()
                    return True

            # Not found within limit
            self._schedule_idle_action()
            return False
        except KeyboardInterrupt:
            self._schedule_idle_action()
            return False

    def goto_minutes(self, minutes_since_midnight: float, speed_sps=None):
        """
        Move to absolute position for the given minutes since midnight (float allowed).
        Steps are cast to int for the final move.
        """
        if speed_sps is None:
            speed_sps = self.run_speed_sps

        target_abs = self._minutes_to_abs_steps(minutes_since_midnight)
        with self._pos_lock:
            delta = target_abs - self._abs_pos_steps
        if delta == 0:
            # no motion, but still enforce idle policy
            self._schedule_idle_action()
            return 0

        self._move_steps_trapezoid(delta, speed_sps)
        with self._pos_lock:
            self._abs_pos_steps += delta
        return abs(int(delta))

    def readback_ihold_irun(self):
        """
        Optional: read IHOLD_IRUN register (0x10) if your driver exposes a read API.
        Returns an int or None.
        """
        if self.driver is None:
            return None
        for name in ("read_register","readRegister","read_reg","readReg","reg_read","register_read"):
            try:
                return int(getattr(self.driver, name)(IHOLD_IRUN_ADDR))
            except Exception:
                pass
        return None

    def cleanup(self):
        """Disable motor and release GPIO."""
        try:
            self.motor_off()
        finally:
            GPIO.cleanup([self.STEP_PIN, self.MINUTE_HOME_PIN, self.HOUR_HOME_PIN])


# ----------------------------------------------------------------------------
# Example usage (uncomment to test as a script)
# ----------------------------------------------------------------------------
if __name__ == "__main__":
    clock = ClockStepperController(
        serial_port="/dev/serial0",  # Prefer ttyAMA0 if you've disabled BT + enabled PL011
        uart_addr=0,
        # Defaults already match your wiring:
        # step_pin=5, dir_pin=6, enable_pin=4,
        # minute_home_pin=26, hour_home_pin=24,
        steps_per_rev=200,
        microsteps=16,
        run_speed_sps=800,
        home_speed_sps=120,
        accel_sps2=2000,
        invert_dir=False,
        enable_low_is_on=True,
        tmc_enable_spreadcycle=True,
        irun_percent=40,
        ihold_percent=10,
        ihold_delay=4,
        auto_disable_ms=2000,   # disable or soft idle after 2s
        soft_idle=False,         # keep EN enabled but drop IHOLD at idle
        idle_ihold_percent=1
    )
    try:
        #print("Homing...")
        #ok = clock.home()
        #print("Homed:", ok)

        print("Go to 07:30 (450.0 minutes)")
        moved = clock.goto_minutes(450.0)
        print("Steps moved:", moved)
        print("Now at minutes:", clock.get_position_minutes())

        # After motion, driver will soft-idle (IHOLD lowered) or fully disable based on config
        time.sleep(5)
    finally:
        clock.cleanup()
