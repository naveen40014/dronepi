from pymavlink import mavutil

# Connect to Pixhawk (adjust port if needed)
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=9600)
master.wait_heartbeat()
print(f"? Heartbeat received from system {master.target_system}, component {master.target_component}")

# Set mode to AUTO
mode = 'AUTO'
mode_id = master.mode_mapping()[mode]

print(f"? Sending mode change to {mode} (mode ID: {mode_id})")
master.set_mode(mode_id)

# Wait for ACK
ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
if ack:
    if ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f"? Mode change to {mode} accepted")
        else:
            print(f"? Mode change to {mode} rejected or failed: result {ack.result}")
    else:
        print(f"? Received ACK for command {ack.command}, not mode change")
else:
    print("? No COMMAND_ACK received for mode change")

# Confirm mode from heartbeat
msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
if msg:
    current_mode = mavutil.mode_string_v10(msg)
    print(f"? Current mode: {current_mode}")
else:
    print("? No heartbeat received to confirm mode")
