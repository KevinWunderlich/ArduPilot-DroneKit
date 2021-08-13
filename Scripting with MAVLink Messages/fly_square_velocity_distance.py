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

#function to get waypoint location for a location due north or due east meters from the orginal locations
#Use negative numbers in the 'dNorth' and 'dEast' arguments for south and west, respectively

def get_location_metres(original_location, dNorth, dEast):
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)

    targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)

    return targetlocation;

def get_distance_meters(targetLocation,currentLocation):
	dLat=targetLocation.lat - currentLocation.lat
	dLon=targetLocation.lon - currentLocation.lon

	return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5

def goto(targetLocation):
	distanceToTargetLocation = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)

	vehicle.simple_goto(targetLocation)

	while vehicle.mode.name=="GUIDED":
		currentDistance = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)
		if currentDistance<distanceToTargetLocation*.05:
			print("Reached target waypoint.")
			break
		time.sleep(1)
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

#Function to fly in a square pattern

def fly_square(length):
    dummy_yaw_initializer()
    goto(get_location_metres(vehicle.location.global_relative_frame,length,0))
    condition_yaw(90,1)
    time.sleep(1)
    goto(get_location_metres(vehicle.location.global_relative_frame,0,length))
    condition_yaw(90,1)
    time.sleep(1)
    goto(get_location_metres(vehicle.location.global_relative_frame,-abs(length),0))
    condition_yaw(90,1)
    time.sleep(1)
    goto(get_location_metres(vehicle.location.global_relative_frame,0,-abs(length)))
    condition_yaw(90,1)
    time.sleep(1)

#################
#Main executable#
#################

change_mode('GUIDED')

fly_square(5)
