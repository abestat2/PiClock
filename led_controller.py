"""
APA102 LED Strip Controller
---------------------------
Requires the `apa102` Python module (often used on Raspberry Pi).
Install (typical): pip install apa102-pi

Features:
- Initialize all LEDs off
- Solid on
- Pulse (all LEDs breathe together)
- Chase (wrap-around "circular" chase)
- Threaded animation loop with clean start/stop
"""

import math
import threading
import time
from typing import Tuple, Optional

try:
    # Typical import name for the APA102/Pi library
    from apa102_pi.driver.apa102 import APA102  # type: ignore
except ImportError as e:
    raise ImportError(
        "The 'apa102' module is required. Try: pip install apa102-pi\n"
        "Original error: " + str(e)
    )

GAMMA = 2.2 # perceptual correction for smoother fades

Color = Tuple[int, int, int]  # (R, G, B), 0-255 each


class APA102Controller:
    """
    High-level controller for APA102/DotStar LED strips using the 'apa102' module.
    """

    def __init__(
        self,
        num_leds: int = 14,
        color: Color = (255, 255, 255),
        brightness: float = 1.0,
        order: str = "brg",
        max_fps: int = 120,
        spi_bus: Optional[int] = None,
        spi_device: Optional[int] = None,
    ) -> None:
        """
        Args:
            num_leds: Number of LEDs on the strip.
            color: Default color for animations (R,G,B).
            brightness: Default brightness (0.0 - 1.0). Animation modes may modulate this.
            order: Color order for the strip. Common: 'rgb', 'bgr', 'grb'.
            max_fps: Frame cap for animations.
            spi_bus: Optional SPI bus (library-dependent).
            spi_device: Optional SPI device (library-dependent).
        """
        if not (0.0 <= brightness <= 1.0):
            raise ValueError("brightness must be between 0.0 and 1.0")

        self.num_leds = int(num_leds)
        self._base_color: Color = color
        self._base_brightness = float(brightness)
        self._order = order
        self._max_fps = max(1, int(max_fps))

        # apa102-pi accepts num_led, order; bus/device are optional and differ by lib variant
        if spi_bus is None and spi_device is None:
            self.strip = APA102(num_led=self.num_leds, order=self._order)
        else:
            # Fallback if your variant exposes these kwargs; ignore if not available.
            try:
                self.strip = APA102(
                    num_led=self.num_leds, order=self._order, bus=spi_bus, device=spi_device
                )
            except TypeError:
                self.strip = APA102(num_led=self.num_leds, order=self._order)

        # Animation state
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._mode = "idle"  # 'idle', 'solid', 'pulse', 'chase'
        self._lock = threading.RLock()

        # Chase parameters
        self._chase_speed = 15.0  # LEDs per second; fractional movement allowed
        self._chase_width = 1.0   # How many LEDs wide is the "head" (can be fractional)
        self._chase_tail = 0.0    # Additional faded tail length (LEDs)

        # Pulse parameters
        self._pulse_period = 2.5  # seconds per full bright->dim->bright cycle
        self._pulse_floor = 0.05  # minimum brightness multiplier (0..1)

        # Initialize all off
        self.all_off()

    # ---------------------------- Public API ---------------------------- #

    def set_color(self, color: Color) -> None:
        """Set base color used by animations."""
        with self._lock:
            self._base_color = tuple(int(max(0, min(255, c))) for c in color)  # type: ignore

    def set_brightness(self, brightness: float) -> None:
        """Set base brightness used by animations (0..1)."""
        if not (0.0 <= brightness <= 1.0):
            raise ValueError("brightness must be between 0.0 and 1.0")
        with self._lock:
            self._base_brightness = float(brightness)

    def set_pulse_params(self, period_s: float = 2.5, floor: float = 0.05) -> None:
        """Adjust pulse behavior."""
        with self._lock:
            self._pulse_period = max(0.1, float(period_s))
            self._pulse_floor = max(0.0, min(1.0, float(floor)))

    def set_chase_params(self, speed_leds_per_s: float = 15.0, width_leds: float = 1.0, tail_leds: float = 0.0) -> None:
        """Adjust chase behavior."""
        with self._lock:
            self._chase_speed = float(speed_leds_per_s)
            self._chase_width = max(0.1, float(width_leds))
            self._chase_tail = max(0.0, float(tail_leds))

    def all_off(self) -> None:
        """Turn all LEDs off immediately and stop animations."""
        self.stop()
        self.strip.clear_strip()
        self.strip.show()

    def solid_on(self, color: Optional[Color] = None, brightness: Optional[float] = None) -> None:
        """Turn all LEDs on continuously, then stop animations (static)."""
        self.stop()
        with self._lock:
            if color is not None:
                self._base_color = color
            if brightness is not None:
                self._base_brightness = brightness
            r, g, b = self._base_color
            level = max(0.0, min(1.0, self._base_brightness))
        for i in range(self.num_leds):
            self.strip.set_pixel(i, r, g, b, int(level * 31))
        self.strip.show()

    def start_pulse(self) -> None:
        """Start the pulse (breathing) animation for all LEDs."""
        self._start_mode("pulse")

    def start_chase(self) -> None:
        """Start the wrap-around chase animation."""
        self._start_mode("chase")

    def start_solid(self) -> None:
        """Start 'solid' mode as a managed mode (keeps thread, allows dynamic changes)."""
        self._start_mode("solid")
        
    def start_group_pulse(self) -> None:
        """Groups of 3 LEDs pulse together"""
        self._start_mode("group_pulse")

    def stop(self) -> None:
        """Stop any running animation thread (if any)."""
        if self._thread and self._thread.is_alive():
            self._stop_event.set()
            self._thread.join(timeout=2.0)
        self._thread = None
        self._stop_event.clear()
        self._mode = "idle"

    def cleanup(self) -> None:
        """Release resources; call before your program exits."""
        self.stop()
        self.strip.clear_strip()
        self.strip.show()
        try:
            self.strip.cleanup()
        except Exception:
            pass

    # ------------------------- Internal methods ------------------------- #

    def _start_mode(self, mode: str) -> None:
        # If already in this mode, ignore
        if self._mode == mode and self._thread and self._thread.is_alive():
            return

        # Stop any current animation thread
        self.stop()

        self._mode = mode
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._animate_loop, name=f"APA102-{mode}", daemon=True)
        self._thread.start()

    def _apply_level(self, color, level):
        # level is 0..1; apply gamma and scale RGB channels
        # keep GB fixed at 31 to avoid 5-bit stepping
        level = max(0.0, min(1.0, level))
        level_gamma = level ** GAMMA
        r, g, b = color
        return(
            int(round(r * level_gamma)),
            int(round(g * level_gamma)),
            int(round(b * level_gamma)),
            31 # fixed GB (global brightness)
        )

    def _animate_loop(self) -> None:
        last_t = time.perf_counter()
        t0 = last_t
        phase = 0.0  # for chase

        # Basic frame limiter
        min_frame_dt = 1.0 / float(self._max_fps)

        while not self._stop_event.is_set():
            now = time.perf_counter()
            dt = now - last_t
            if dt < min_frame_dt:
                # Sleep only the remaining time slice
                time.sleep(min_frame_dt - dt)
                now = time.perf_counter()
                dt = now - last_t
            last_t = now

            with self._lock:
                mode = self._mode
                color = self._base_color
                base_brightness = self._base_brightness
                pulse_period = self._pulse_period
                pulse_floor = self._pulse_floor
                chase_speed = self._chase_speed
                chase_width = self._chase_width
                chase_tail = self._chase_tail

            if mode == "solid":
                r, g, b = color
                br = int(max(0, min(31, round(base_brightness * 31))))
                for i in range(self.num_leds):
                    self.strip.set_pixel(i, r, g, b, br)
                self.strip.show()
                # sleep a tad to avoid pegging CPU
                continue

            elif mode == "pulse":
                # Breathing brightness: cosine from 0..1 scaled to floor..1
                elapsed = now - t0
                # cos oscillates 1..-1; we convert to 0..1
                osc = 0.5 * (1.0 + math.cos(2 * math.pi * (elapsed / pulse_period)))
                level = pulse_floor + (1.0 - pulse_floor) * osc
                r, g, b, br = self._apply_level(color, level * base_brightness) 
                for i in range(self.num_leds):
                    self.strip.set_pixel(i, r, g, b, br)
                self.strip.show()

            elif mode == "chase":
                # Phase advances in LEDs per second
                phase = (phase + chase_speed * dt) % self.num_leds
                r, g, b = color

                # Precompute head/tail profile
                # Create a soft head with optional tail fade.
                for i in range(self.num_leds):
                    # Distance along the ring from the head (wrap-around shortest distance)
                    d = self._circular_distance(i, phase, self.num_leds)

                    # Head region (full brightness within width/2)
                    head_edge = 0.5 * chase_width
                    if d <= head_edge:
                        gain = 1.0
                    else:
                        # Tail falloff beyond the head up to head+tail
                        if chase_tail > 0.0 and d <= head_edge + chase_tail:
                            # Simple linear fade to 0
                            gain = max(0.0, 1.0 - (d - head_edge) / max(chase_tail, 1e-6))
                        else:
                            gain = 0.0

                    br = int(max(0, min(31, round(gain * base_brightness * 31))))
                    if br > 0:
                        self.strip.set_pixel(i, r, g, b, br)
                    else:
                        self.strip.set_pixel(i, 0, 0, 0, 0)

                self.strip.show()
            elif mode == "group_pulse":
                elapsed = now - t0
                osc = 0.5 * (1.0 + math.cos(2 * math.pi * (elapsed / pulse_period)))
                level = pulse_floor + (1.0 - pulse_floor) * osc
                # pulse in groups of 3 LEDs
                r, g, b, br = self._apply_level(color, level * base_brightness) 
                for i in range(self.num_leds):
                    if (i % 3) == 0:
                        self.strip.set_pixel(i, r, g, b, br)
                    else:
                        self.strip.set_pixel(i, 0, 0, 0, 0)
                self.strip.show()
            else:
                # Idle -> ensure off
                self.strip.clear_strip()
                self.strip.show()
                time.sleep(0.02)

    @staticmethod
    def _circular_distance(i: float, head_pos: float, n: int) -> float:
        """
        Shortest circular distance in LED units between index i and a floating head_pos.
        """
        # Normalize
        i = float(i)
        head_pos = float(head_pos)
        n = int(n)
        # Compute forward/backward distances on the ring
        forward = (i - head_pos) % n
        backward = (head_pos - i) % n
        return min(forward, backward)


# ----------------------------- Example usage ----------------------------- #
if __name__ == "__main__":
    """
    Example:
        python led_controller.py

    Edit the parameters below for your strip.
    """
    NUM_LEDS = 43
    DEFAULT_COLOR = (255, 255, 255)  # teal-ish
    BRIGHTNESS = 1.0

    ctrl = APA102Controller(NUM_LEDS, color=DEFAULT_COLOR, brightness=BRIGHTNESS, order="rgb", max_fps=120, spi_bus=0, spi_device=0) 

    try:
        print("All off for 1s...")
        ctrl.all_off()
        time.sleep(1)

        print("Solid on (3s)...")
        ctrl.solid_on()
        time.sleep(3)

        print("Pulse for 6s...")
        ctrl.start_pulse()
        time.sleep(6)

        print("Pulse group for 6s...")
        ctrl.start_group_pulse()
        time.sleep(6)

        print("Chase for 6s...")
        ctrl.set_chase_params(speed_leds_per_s=10.0, width_leds=2, tail_leds=3)
        ctrl.start_chase()
        time.sleep(6)

        print("Back to solid (3s)...")
        ctrl.start_solid()
        time.sleep(3)

    finally:
        print("Cleanup and off.")
        ctrl.cleanup()
