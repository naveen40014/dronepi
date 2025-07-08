from pymavlink import mavutil

# Connect to Pixhawk over USB (adjust port if needed)
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=9600)

# Wait for the heartbeat
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"? Heartbeat from system {master.target_system} component {master.target_component}")

# Receive and print 5 MAVLink messages
for _ in range(5):
    msg = master.recv_match()
    if msg:
        print(msg)
