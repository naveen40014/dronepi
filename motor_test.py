from pymavlink import mavutil
import time

SERIAL_PORT = "/dev/ttyACM0"  # USB device
BAUDRATE    = 115200
MOTOR_NUM   = 1               # Starting motor number
THROTTLE    = 20              # Throttle percent (10–100)
DURATION    = 5               # Duration per motor
MOTOR_COUNT = 4               # Total motors (adjust if hexacopter = 6)

def motor_test(master, motor_number, throttle_percent, duration_sec):
    print(f"> Testing Motor {motor_number} at {throttle_percent}% for {duration_sec}s")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,                     # Confirmation
        motor_number - 1,      # motor number (0-indexed)
        0,                     # PWM test
        throttle_percent,
        duration_sec,
        0, 0, 0
    )

def main():
    master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUDRATE)
    master.wait_heartbeat()
    print("✓ Connected to flight controller")

    for i in range(1, MOTOR_COUNT + 1):
        motor_test(master, i, THROTTLE, DURATION)
        time.sleep(DURATION + 1)

    print("✓ All motors tested")

if __name__ == "__main__":
    main()
