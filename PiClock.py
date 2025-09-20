import time
import socket
from datetime import datetime
from pathlib import Path

import ntplib

from led_controller import APA102Controller
from motor_controller import ClockStepperController


# ------------------- Utility Checks ------------------- #
def check_wifi():
    """Check if WiFi is connected (ssid available)."""
    try:
        import subprocess
        result = subprocess.run(["iwgetid"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return result.stdout != b""
    except Exception:
        return False

def check_internet(host="8.8.8.8", port=53, timeout=3):
    """Check if internet (raw IP reachability) works."""
    try:
        socket.setdefaulttimeout(timeout)
        socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect((host, port))
        return True
    except Exception:
        return False

def check_dns(host="www.google.com"):
    """Check DNS resolution works."""
    try:
        socket.gethostbyname(host)
        return True
    except Exception:
        return False

def get_time_from_ntp(ntp_servers_file="ntp_servers.txt"):
    """Try to get time from NTP servers listed in file."""
    servers_file = Path(ntp_servers_file)
    if not servers_file.exists():
        print("NTP servers file missing")
        return None

    servers = [line.strip() for line in servers_file.read_text().splitlines() if line.strip()]
    client = ntplib.NTPClient()

    for server in servers:
        try:
            response = client.request(server, version=3)
            # Convert NTP timestamp to local datetime
            return datetime.utcfromtimestamp(response.tx_time) + (
                datetime.now() - datetime.utcnow()
            )
        except Exception as e:
            print(f"NTP {server} failed: {e}")
    return None


# ------------------- Main State Machine ------------------- #
def main():
    leds = APA102Controller(num_leds=14, color=(0, 255, 180), brightness=0.3, order="rgb")
    clock = ClockStepperController()

    homed = False  # Track whether homing was done

    try:
        while True:
            # --- 1. WiFi check ---
            if not check_wifi():
                leds.start_group_pulse()
                time.sleep(15)
                continue

            # --- 2. Internet check ---
            if not check_internet():
                leds.start_pulse()
                time.sleep(15)
                continue

            # --- 3. DNS check ---
            if not check_dns():
                leds.all_off()
                time.sleep(15)
                continue

            # --- 4. NTP sync ---
            ntp_time = get_time_from_ntp()
            if ntp_time is None:
                leds.start_chase()
                time.sleep(15)
                continue

            # --- 5. Time success: update motors ---
            print("Time sync success:", ntp_time)

            # Run homing only once
            if not homed:
                print("Homing clock hands...")
                ok = clock.home()
                print("Homed:", ok)
                homed = True

            # Run for 1 hour, updating motor each second
            start = time.time()
            while (time.time() - start) < 3600:
                now = datetime.now()
                minutes = now.hour * 60 + now.minute + now.second / 60.0
                clock.goto_minutes(minutes)
                time.sleep(1)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        leds.cleanup()
        clock.cleanup()


if __name__ == "__main__":
    main()
