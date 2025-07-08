from pymavlink import mavutil
import time
import sys

# Connect to the Pixhawk (adjust port and baud if needed)
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=9600)

print("🔄 Waiting for heartbeat...")
sys.stdout.flush()

master.wait_heartbeat()
print(f"✅ Heartbeat received from system {master.target_system}")
sys.stdout.flush()

# Set mode to GUIDED (mode ID 4 for ArduCopter)
mode = 'GUIDED'
mode_id = master.mode_mapping()[mode]

# Send mode change request
master.set_mode(mode_id)
print(f"➡️ Sent mode change request to {mode}")
sys.stdout.flush()

# Wait and check mode confirmation
time.sleep(2)
print("✅ Mode change command sent. Check Mission Planner or telemetry for confirmation.")
sys.stdout.flush()
