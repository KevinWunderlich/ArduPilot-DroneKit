##########DEPENDENCIES#############

from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
import time
import socket
import exceptions
import math
import argparse
from pymavlink import mavutil

vehicle = connect('tcp:127.0.0.1:5763',wait_ready=True)

##Function to Change Flight Mode

def change_mode(des_mode):
    vehicle.mode = VehicleMode(des_mode)
    while vehicle.mode != des_mode:
        print("Waiting for mode change")
        time.sleep(1)
    print("Mode change succesful")
    return None

#Function to send a velocity message in a local frame
#+x is heading of the drone
#arguments are in m/s

def send_local_ned_velocity(vx, vy, vz):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,
		0, 0,
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
		0b0000111111000111,
		0, 0, 0,
		vx, vy, vz,
		0, 0, 0,
		0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush()

#Function for yaw rotation

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

#yaw initializer

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

##Function to fly in a square with sides of a specified length

def fly_square2(length):
	print("Flying in square flight pattern")
	dummy_yaw_initializer()
	time.sleep(1)
	turns = 0
	while turns < 4:
		counter = 0
		while counter<(length*2):
			send_local_ned_velocity(.5,0,0)
			time.sleep(1)
			counter = counter + 1
		time.sleep(2)
		condition_yaw(90,1)
		turns = turns + 1
	return None

####Function to fly in a rectangle pattern with sides of a specified length and width

def fly_rectangle2(length,width):
	print("Flying in rectangle flight pattern")
	dummy_yaw_initializer()
	time.sleep(1)
	turns = 0
	while turns < 4:
		counter = 0
		if turns % 2 == 0:
			while counter<(length*2):
				send_local_ned_velocity(.5,0,0)
				time.sleep(1)
				counter = counter + 1
			time.sleep(2)
			condition_yaw(90,1)
			turns = turns + 1
		else:
			while counter<(width*2):
				send_local_ned_velocity(.5,0,0)
				time.sleep(1)
				counter = counter + 1
			time.sleep(2)
			condition_yaw(90,1)
			turns = turns + 1
	return None

##########MAIN EXECUTABLE###########

change_mode('GUIDED')
fly_square2(5)
time.sleep(2)
fly_rectangle2(5,2)
