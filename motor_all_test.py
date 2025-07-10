from pymavlink import mavutil
import time

# ✅ CONFIGURATION
SERIAL_PORT = "/dev/serial0"   # or "/dev/ttyACM0" for USB
BAUDRATE    = 57600            # Match TELEM2 baudrate
THROTTLE    = 20               # Percent (10–100%)
DURATION    = 5                # Seconds

def motor_test_all(master, throttle_percent, duration_sec):
    print(f"> Testing ALL motors at {throttle_percent}% for {duration_sec}s")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,                     # Confirmation
        255,                   # Motor number 255 = ALL motors
        0,                     # Test type = 0 (PWM Percent)
        throttle_percent,      # Throttle %
        duration_sec,          # Duration (s)
        0, 0, 0                # Unused
    )

def main():
    master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUDRATE)
    master.wait_heartbeat()
    print("✓ Connected to flight controller")

    motor_test_all(master, THROTTLE, DURATION)

    print(f"✓ Test running... waiting {DURATION} seconds")
    time.sleep(DURATION + 1)
    print("✓ Done")

if __name__ == "__main__":
    main()
