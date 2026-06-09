import time
import subprocess
import argparse
import math
import numpy as np
from scipy.signal import max_len_seq
from pymavlink import mavutil

# Helper function for emptying buffer and keeping connection active
def safe_delay(delay, mav_connection):
    t_start = time.time()
    while(time.time() - t_start < delay):
        mav_connection.recv_match(blocking=False)

# Function to calculate roll angle into quaternion format
def roll_2_quaternion(angle):
    half_angle = angle / 2.0
    w = math.cos(half_angle)
    x = math.sin(half_angle) # FIXED: Changed from cos to sin for X-axis
    y = 0.0 
    z = 0.0
    return [w, x, y, z]
    
# Compressing noise sending into one function
def execute_roll_noise(duration, roll_seq, mav_connection, freq):
    start_time = time.time()
    last_time = 0.0
    ts = 1.0 / freq
    i = 0
    seq_len = len(roll_seq)
    
    while (time.time() - start_time < duration):
        current_time = time.time()
        if(current_time - last_time >= ts):
            q = roll_2_quaternion(roll_seq[i % seq_len])
            mav_connection.mav.set_attitude_target_send(
                0,
                mav_connection.target_system,
                mav_connection.target_component,
                7,
                q,
                0, 0, 0,
                0.55
            )
            last_time = current_time
            i += 1 
            
        mav_connection.recv_match(blocking=False)

# Preparing args parser for easier adjustments of the tests
parser = argparse.ArgumentParser(description="Parser for roll validation")
parser.add_argument("--noise_freq", type=float, default=5.0, help="Frequency of injected noise")
parser.add_argument("--noise_dur", type=float, default=20.0, help="Duration of noise injecting")
parser.add_argument("--stab_dur", type=float, default=10.0, help="Duration of stabilization phases")
parser.add_argument("--alt", type=float, default=2.0, help="Altitude at which test will be started")
args = parser.parse_args()

# Preparing noise signal varying between -10deg to 10deg
minimal_noise_samples = math.floor(args.noise_dur / (1.0 / args.noise_freq)) + 1
i = 1
while((minimal_noise_samples) > (2.0**i - 1)):
    i += 1 # FIXED: Was i += i
noise = max_len_seq(i)[0].astype(float) 
roll_amplitude_rad = 5.0 / 360.0 * 2.0 * math.pi

for idx in range(len(noise)): 
    if(noise[idx] == 0):
        noise[idx] = -roll_amplitude_rad
    elif (noise[idx] == 1):
        noise[idx] = roll_amplitude_rad

# Connect to the simulator
connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# Wait for the first heartbeat
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

try:
    mode_id = connection.mode_mapping()['GUIDED']
except Exception as e:
    print("Cannot download mode id from flight controller")
    exit()

# Arming loop
while not connection.motors_armed():
    connection.set_mode(mode_id)
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("System disarmed, waiting for system setup...")
    safe_delay(2.0, connection)

# Send autonomous takeoff command
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 
    args.alt
)
print(f"Takeoff command sent. Ascending to {args.alt} meters.")
safe_delay(args.stab_dur, connection)

print("Phase: Injecting PRBS Noise (Payload Attached)")
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: NOISE_1_START")
execute_roll_noise(args.noise_dur, noise, connection, args.noise_freq)
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: NOISE_1_STOP") # FIXED: STOP instead of START

# FIXED: Bring drone to level flight and stabilize BEFORE dropping payload
print("Phase: Leveling and stabilizing before drop")
connection.mav.set_position_target_local_ned_send(
    0,                                  # time_boot_ms
    connection.target_system,           
    connection.target_component,        
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,# coordinate_frame
    2552,                               # type_mask (ignore velocities and accelerations)
    0, 0, -args.alt,                    # x, y, z (z is negative in NED)
    0, 0, 0,                            # vx, vy, vz
    0, 0, 0,                            # afx, afy, afz
    0, 0                                # yaw, yaw_rate
)
safe_delay(20.0, connection)

print("Phase: Payload drop")
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: PAYLOAD_DROP")
try:
    subprocess.run(
        ["gz", "topic", "-t", "/payload/detach", "-m", "gz.msgs.Empty", "-p", " "],
        check=True
    )
    print("Payload detached successfully.")
except Exception as e:
    print("Error during payload detachment:", e)
    
print("Phase: Stabilization after payload drop")
safe_delay(args.stab_dur, connection)

print("Phase: Injecting PRBS Noise (Payload Detached)")
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: NOISE_2_START")
execute_roll_noise(args.noise_dur, noise, connection, args.noise_freq)
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: NOISE_2_STOP") # FIXED: STOP instead of START

# Final stabilization before script ends
connection.mav.set_position_target_local_ned_send(
    0,                                  # time_boot_ms
    connection.target_system,           
    connection.target_component,        
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,# coordinate_frame
    2552,                               # type_mask (ignore velocities and accelerations)
    0, 0, -args.alt,                    # x, y, z (z is negative in NED)
    0, 0, 0,                            # vx, vy, vz
    0, 0, 0,                            # afx, afy, afz
    0, 0                                # yaw, yaw_rate
)
safe_delay(3.0, connection)
print("Identification routine completed.")