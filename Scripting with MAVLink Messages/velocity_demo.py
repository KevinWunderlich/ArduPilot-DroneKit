##########DEPENDENCIES#############

from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
import time
import socket
import exceptions
import math
import argparse
from pymavlink import mavutil

vehicle = connect('tcp:127.0.0.1:5763',wait_ready=True)

#########FUNCTIONS#################

##Function to Change Flight Mode

def change_mode(des_mode):
    vehicle.mode = VehicleMode(des_mode)
    while vehicle.mode != des_mode:
        print("Waiting for mode change")
        time.sleep(1)
    print("Mode change succesful")
    return None

##Send a velocity command with +x being the heading of the drone.

def send_local_ned_velocity(vx, vy, vz):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0, # time_boot_ms (not used)
		0, 0, # target system, target component
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, #frame
		0b0000111111000111, #type_mask (only speeds enabled)
		0, 0, 0, # x, y, z positions (not used)
		vx, vy, vz, # x, y, z velocity in m/s
		0, 0, 0, #x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0) #yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
	vehicle.send_mavlink(msg)
	vehicle.flush()

##Send a velocity command with +x being true north
def send_global_ned_velocity(vx, vy, vz):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0, # time_boot_ms (not used)
		0, 0, # target system, target component
		mavutil.mavlink.MAV_FRAME_LOCAL_NED, #frame
		0b0000111111000111, #type_mask (only speeds enabled)
		0, 0, 0, # x, y, z positions (not used)
		vx, vy, vz, # x, y, z velocity in m/s
		0, 0, 0, #x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0) #yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
	vehicle.send_mavlink(msg)
	vehicle.flush()

##########MAIN EXECUTABLE###########

change_mode('GUIDED')

counter=0
while counter<5:
	send_local_ned_velocity(.5,0,0)
	time.sleep(1)
	print("Moving NORTH relative to front of drone")
	counter=counter+1

time.sleep(2)

counter=0
while counter<5:
	send_local_ned_velocity(0,-.5,0)
	time.sleep(1)
	print("Moving WEST relative to front of drone")
	counter=counter+1

time.sleep(2)

counter = 0
while counter < 5:
	send_local_ned_velocity(0,.5,-.5)
	time.sleep(1)
	print("Moviing EAST relative to front of drone and UP in altitude")
	counter = counter + 1

time.sleep(2)

counter = 0
while counter < 5:
	send_local_ned_velocity(-.5,-.5,.5)
	time.sleep(1)
	print("Moving SOUTHWEST relative to front of drone and DOWN in altitude ")
	counter = counter + 1
