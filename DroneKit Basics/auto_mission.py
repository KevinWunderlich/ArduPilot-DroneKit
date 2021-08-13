##########DEPENDENCIES#############

from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException,Command
import time
import socket
import exceptions
import math
import argparse
from pymavlink import mavutil

#####Connection String

vehicle = connect('tcp:127.0.0.1:5763',wait_ready=True)

##Functions

def change_mode(des_mode):
    vehicle.mode = VehicleMode(des_mode)
    while vehicle.mode != des_mode:
        print("Waiting for mode change")
        time.sleep(1)
    print("Mode change succesful")
    return None

def arm_vehicle():
    vehicle.armed = True
    while vehicle.armed==False:
        print("Waiting for vehicle to become armed.")
        time.sleep(1)
    print("Vehicle armed.")
    return None

def arm_and_takeoff(targetHeight):
	while vehicle.is_armable!=True:
		print("Waiting for vehicle to become armable.")
		time.sleep(1)
	print("Vehicle is now armable")

	change_mode('GUIDED')

	arm_vehicle()

	vehicle.simple_takeoff(targetHeight) ##meters

	while True:
		print("Current Altitude: %d"%vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt>=.91*targetHeight:
			break
		time.sleep(1)
	print("Target altitude reached.")

	return None


#################
#Main Executable#
#################

##Command template
#Command(0,0,0,FrameOfReference,MAVLinkCommand,CurrentWP,AutoContinue,param1,param2,param3,param4,param5,parm6,param7)

wphome=vehicle.location.global_relative_frame
wp2 = LocationGlobalRelative(40.655410,-73.970752,15)
wp3 = LocationGlobalRelative(40.655556,-73.969050,10)

##List of commands
cmd1=Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,wphome.lat,wphome.lon,wphome.alt)
cmd2=Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,wp2.lat,wp2.lon,wp2.alt)
cmd3=Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,wp3.lat,wp3.lon,10)
cmd4=Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,0,0,0,0,0,0,0,0,0)

##Download current list of commands FROM the drone we're connected to
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

##Clear the current list of commands
cmds.clear()

##Add in our new commands
cmds.add(cmd1)
cmds.add(cmd2)
cmds.add(cmd3)
cmds.add(cmd4)

##Upload our commands to the drone
vehicle.commands.upload()

arm_and_takeoff(10)

print("After arm and takeoff")
vehicle.mode = VehicleMode("AUTO")
while vehicle.mode!="AUTO":
	time.sleep(.2)

counter = 0

while vehicle.location.global_relative_frame.alt>2:
    if counter % 10 == 0:
        print('Drone has been performing the mission for %s seconds'%counter)
    counter = counter + 2
    time.sleep(2)
