##########DEPENDENCIES#############

from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
import time
import socket
import exceptions
import math
import argparse
from pymavlink import mavutil

##Connection String

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

def condition_yaw(degrees,relative):
	if relative:
		is_relative = 1 #yaw relative to direction of travel
	else:
		is_relative = 0 #yaw is an absolute angle

	# create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
		0, 0, # target system, target component
		mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
		0, #confirmation
		degrees, #Param 1, yaw in degrees
		0,  #Param 2, yaw speed deg/s
		1, #Param 3, Direction -1 ccw, 1 cw
		is_relative, # Param 4, relative offset 1, absolute angle 0
		0, 0, 0) # Param 5-7 not used
	vehicle.send_mavlink(msg)
	vehicle.flush()

#Function to initizalie Yaw attitude

def dummy_yaw_initializer():
	lat=vehicle.location.global_relative_frame.lat
	lon=vehicle.location.global_relative_frame.lon
	alt=vehicle.location.global_relative_frame.alt

	aLocation=LocationGlobalRelative(lat,lon,alt)

	msg = vehicle.message_factory.set_position_target_global_int_encode(
		0,
		0, 0,
		mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
		0b0000111111111000,
		aLocation.lat*1e7,
		aLocation.lon*1e7,
		aLocation.alt,
		0,
		0,
		0,
		0, 0, 0,
		0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush()

##########MAIN EXECUTABLE###########

change_mode('GUIDED')

dummy_yaw_initializer()
time.sleep(2)

condition_yaw(30,1) ##180 -->> 210
print("Yawing 30 degrees relative to current position")
time.sleep(7)

print("Yawing True North")
condition_yaw(0,0) ##Yaw to true north
time.sleep(7)
print("Yawing True West")
condition_yaw(270,0) ##Yaw to true west

while True:
	time.sleep(1)
