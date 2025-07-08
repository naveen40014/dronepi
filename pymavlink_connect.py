from pymavlink import mavutil

# Adjust your port if needed
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=9600)

print("?? Waiting for heartbeat from Pixhawk...")

# Wait for heartbeat
master.wait_heartbeat()

print(f"? Connected! Heartbeat received from system {master.target_system}, component {master.target_component}")

# Optionally print a few incoming messages to confirm live telemetry
for _ in range(5):
    msg = master.recv_match()
    if msg:
        print(msg)
