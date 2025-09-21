# motor_controller.py 
import time
import threading
import RPi.GPIO as GPIO

_tmc2209_cls = None
try:
    # Adjust to match your installed lib
    from TMC2209 import TMC2209
    _tmc2209_cls = TMC2209
except Exception:
    try:
        from tmc2209 import TMC2209
        _tmc2209_cls = TMC2209
    except Exception:
        _tmc2209_cls = None


class ClockStepperController:
    def __init__(
        self,
        *,
        serial_port="/dev/serial0",
        uart_addr=0,
        step_pin=18,
        dir_pin=23,
        enable_pin=24,
        minute_home_pin=5,
        hour_home_pin=6,
        steps_per_rev=200,            # full steps
        minute_per_motor_rev=10/3,    # given
        hour_per_minute_rev=1/18,     # given
        microsteps=16,                # hardware pins or UART (optional)
        run_speed_sps=800,            # cruise speed (steps/sec)
        home_speed_sps=120,           # slower to avoid buzzing
        accel_sps2=2000,              # accel/decel slope
        debounce_ms=8,
        enable_low_is_on=True,
        invert_dir=False,
        step_pulse_us=10,             # wider STEP pulse
        step_low_us=10,               # enforce low time too
        dir_setup_us=60,              # time to settle DIR before stepping
        enable_settle_ms=5,           # wait after enabling driver
        tmc_enable_spreadcycle=True,  # switch off stealthChop for homing/low speed reliability
        irun_percent=35,              # run current % (adjust for your motor/driver)
        ihold_percent=15              # hold current %
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

        self._abs_pos_steps = 0
        self._pos_lock = threading.Lock()

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.STEP_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.DIR_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.ENABLE_PIN, GPIO.OUT, initial=GPIO.HIGH if self.enable_low_is_on else GPIO.LOW)
        GPIO.setup(self.MINUTE_HOME_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.HOUR_HOME_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self._last_minute_state = (GPIO.input(self.MINUTE_HOME_PIN) == GPIO.LOW)
        self._last_hour_state = (GPIO.input(self.HOUR_HOME_PIN) == GPIO.LOW)
        self._last_minute_time = time.time()
        self._last_hour_time = time.time()

        self.driver = None
        self._configure_driver()

    # ------------ Driver / GPIO helpers ------------

    def _configure_driver(self):
        if _tmc2209_cls is None:
            return
        try:
            # Your library’s ctor may differ; adapt as needed.
            self.driver = _tmc2209_cls(serial_port=self.serial_port, addr=self.uart_addr)
        except Exception:
            self.driver = None

        if self.driver:
            # Try to set currents / mode if API exists. Silently ignore if not.
            for method, args in [
                ("set_run_current", (self.irun_percent,)),
                ("set_hold_current", (self.ihold_percent,)),
                ("set_microstepping", (self.microsteps,)),
            ]:
                try:
                    getattr(self.driver, method)(*args)
                except Exception:
                    pass
            if self.tmc_enable_spreadcycle:
                for method, args in [
                    ("enable_spreadcycle", (True,)),
                    ("enable_stealth_chop", (False,)),
                ]:
                    try:
                        getattr(self.driver, method)(*args)
                    except Exception:
                        pass

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
        # Allow DIR to settle before the first STEP edge
        time.sleep(self.dir_setup_us / 1_000_000.0)

    def _pulse_step(self):
        GPIO.output(self.STEP_PIN, GPIO.HIGH)
        time.sleep(self.step_pulse_us / 1_000_000.0)
        GPIO.output(self.STEP_PIN, GPIO.LOW)
        time.sleep(self.step_low_us / 1_000_000.0)

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

    # ------------ Mapping: minutes -> steps ------------

    def _minutes_to_abs_steps(self, minutes_since_midnight: float) -> int:
        # motor_revs = minutes / 200, steps = steps_per_rev * motor_revs
        steps_float = self.steps_per_rev * (float(minutes_since_midnight) / 200.0)
        return int(steps_float)  # per your requirement

    # ------------ Motion primitives (with acceleration) ------------

    def _move_steps_trapezoid(self, steps: int, vmax_sps: float):
        """Signed steps with a simple accel/decel profile."""
        if steps == 0:
            return
        forward = steps > 0
        n = abs(int(steps))

        self._enable_motor(True)
        self._set_dir(forward)

        # Compute steps to accelerate: v^2 = 2*a*s  => s = v^2 / (2a)
        a = max(self.accel_sps2, 1.0)
        vmax = max(vmax_sps, 50.0)
        s_accel = int(max(1, round((vmax * vmax) / (2.0 * a))))
        s_accel = min(s_accel, n // 2)  # symmetric accel/decel
        s_cruise = n - 2 * s_accel

        # Accel phase
        v = 50.0  # start gently
        for i in range(s_accel):
            # increase speed linearly
            v = min(vmax, v + a / max(v, 1.0))  # simple ramp
            dt = 1.0 / v
            self._pulse_step()
            # dt already includes step_low_us, so subtract the pulse width portion
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

        # Decel phase (mirror accel)
        for i in range(s_accel, 0, -1):
            # decrease speed
            # approximate symmetric ramp
            v = max(50.0, (i * a) ** 0.5)  # keep >0
            v = min(v, vmax)
            dt = 1.0 / v
            self._pulse_step()
            remaining = max(0.0, dt - (self.step_pulse_us + self.step_low_us)/1_000_000.0)
            if remaining > 0:
                time.sleep(remaining)

        self._enable_motor(False)

    # ------------ Public API ------------

    def home(self, max_search_steps=4000):
        """Rotate forward at homing speed until BOTH switches are active simultaneously."""
        self._enable_motor(True)
        self._set_dir(True)  # forward
        v = max(self.home_speed_sps, 60.0)
        dt = 1.0 / v
        base_sleep = max(0.0, dt - (self.step_pulse_us + self.step_low_us)/1_000_000.0)

        # settle debounce
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
                    self._enable_motor(False)
                    return True
            self._enable_motor(False)
            return False
        except KeyboardInterrupt:
            self._enable_motor(False)
            return False

    def goto_minutes(self, minutes_since_midnight: float, speed_sps=None):
        if speed_sps is None:
            speed_sps = self.run_speed_sps
        target_abs = self._minutes_to_abs_steps(minutes_since_midnight)
        with self._pos_lock:
            delta = target_abs - self._abs_pos_steps
        if delta == 0:
            return 0
        self._move_steps_trapezoid(delta, speed_sps)
        with self._pos_lock:
            self._abs_pos_steps += delta
        return abs(int(delta))

    def get_position_minutes(self) -> float:
        with self._pos_lock:
            steps = self._abs_pos_steps
        return 200.0 * steps / float(self.steps_per_rev)

    def cleanup(self):
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
        microsteps=16,             # optional, if configured in your driver
        run_speed_sps=400,
        home_speed_sps=80,
        debounce_ms=8,
        enable_low_is_on=True,
        invert_dir=False,
        step_pulse_us=10,
        dir_setup_us=60,
        enable_settle_ms=5,
        tmc_enable_spreadcycle=True,
        irun_percent=35,
        ihold_percent=15
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
