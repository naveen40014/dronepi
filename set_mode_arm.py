from pymavlink import mavutil

# Connect to Pixhawk over USB (adjust port if needed)
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=9600)

# Wait for the heartbeat
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"? Heartbeat from system {master.target_system} component {master.target_component}")

# Set mode to GUIDED
master.set_mode_apm('GUIDED')
print("? Sent mode change to GUIDED")

# Arm the drone
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)
print("? Sent arming command")
