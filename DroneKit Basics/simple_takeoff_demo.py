##########DEPENDENCIES#############

from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
import time
import socket
import exceptions
import math
import argparse

##########CONNECTION STRING###########

vehicle = connect('tcp:127.0.0.1:5763',wait_ready=True)

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
    print("Vehicle armed.")
    return None

##########ARM AND TAKEOFF FUNCTION###########

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

##########MAIN EXECUTABLE###########

arm_and_takeoff(10)
vehicle.close()
