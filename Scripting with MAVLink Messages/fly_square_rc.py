##########DEPENDENCIES#############

from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
import time
import socket
import exceptions
import math
import argparse
from pymavlink import mavutil

vehicle = connect('tcp:127.0.0.1:5763',wait_ready=True)

#Reset RC Channel values to neutral

def reset_rc():
    vehicle.channels.overrides = {'1':1500, '2':1500, '3':1500, '4':1500}
    return None

##Function to Change Flight Mode

def change_mode(des_mode):
    vehicle.mode = VehicleMode(des_mode)
    while vehicle.mode != des_mode:
        print("Waiting for mode change")
        time.sleep(1)
    print("Mode change succesful")
    return None

#Wait for vehicle's yaw attitude to reach a certain angle

def wait_yaw(des_yaw):

    while True:
        print("Current yaw in degrees is %d:" %math.degrees(vehicle.attitude.yaw))
        if math.degrees(vehicle.attitude.yaw) >=.98*des_yaw:
            reset_rc()
            break
        time.sleep(.1)

    return None

#####Distance calculator between locations####

def get_distance_meters(targetLocation,currentLocation):
	dLat=targetLocation.lat - currentLocation.lat
	dLon=targetLocation.lon - currentLocation.lon

	return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5

####Function to wait until vehicle travels a desired distance#####

def wait_for_dist(des_dist):
    start_loc = vehicle.location.global_relative_frame #set starting location
    ###Wait for vehicle to reach target distance within a margin of error###
    while True:
        current_dist = get_distance_meters(start_loc,vehicle.location.global_relative_frame) #Calculates current distance from starting location
        print("Current Distance from Starting Location: %d"%current_dist)
        if current_dist>=des_dist*.95:
            print("Desired distance reached")
            break
        time.sleep(1)
    return None

####Function to fly in a Square Pattern with a desired length / width####

def fly_square(length):
    vehicle.channels.overrides['2'] = 1400
    wait_for_dist(length)
    reset_rc()
    time.sleep(1)
    vehicle.channels.overrides['4'] = 1550
    wait_yaw(90)
    vehicle.channels.overrides['2'] = 1400
    wait_for_dist(length)
    reset_rc()
    time.sleep(1)
    vehicle.channels.overrides['4'] = 1550
    wait_yaw(180)
    vehicle.channels.overrides['2'] = 1400
    wait_for_dist(length)
    reset_rc()
    time.sleep(1)
    vehicle.channels.overrides['4'] = 1550
    wait_yaw(-90)
    vehicle.channels.overrides['2'] = 1400
    wait_for_dist(length)
    reset_rc()
    time.sleep(1)
    vehicle.channels.overrides['4'] = 1550
    wait_yaw(0)

#######################
####Main Executable####
#######################

reset_rc()

change_mode('LOITER')

fly_square(5)
