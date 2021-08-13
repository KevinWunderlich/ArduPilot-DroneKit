##########################################################################
#Script to demonstrate programmed flight behavior through virtual RC inputs#
##########################################################################

##########DEPENDENCIES#############

from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
import time
import socket
import exceptions
import math
import argparse
from pymavlink import mavutil

##Connection String###

vehicle = connect('tcp:127.0.0.1:5763',wait_ready=True)

#Reset RC Channel values to neutral

def reset_rc():
    vehicle.channels.overrides = {'1':1500, '2':1500, '3':1500, '4':1500}
    return None

###Function to wait until Vehicle reaches a desired altitude

def wait_for_alt(des_alt):
    while True:
        print("Current Altitude: %d"%vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=.95*des_alt:
            break
        time.sleep(1)
    print("Target altitude reached")

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

##Function to Change Flight Mode

def change_mode(des_mode):
    vehicle.mode = VehicleMode(des_mode)
    while vehicle.mode != des_mode:
        print("Waiting for mode change")
        time.sleep(1)
    print("Mode change succesful")
    return None

##Function to Arm Vehicle##

def arm_vehicle():
    vehicle.armed = True
    while vehicle.armed==False:
        print("Waiting for vehicle to become armed.")
        time.sleep(1)
        print("")
    return None

#######################
####Main Executable####
#######################

reset_rc()
change_mode('LOITER')
arm_vehicle()
#Set throttle level
vehicle.channels.overrides['3'] = 1650
#wait to reach desired altitude
wait_for_alt(5)
reset_rc()
time.sleep(3)
#Set RC Pitch to move forward
vehicle.channels.overrides['2'] = 1400
#Wait for vehicle to reach desired distance
wait_for_dist(5)
reset_rc()
#Return to home location
change_mode('RTL')
