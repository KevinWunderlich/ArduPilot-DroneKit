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

###Get distance in meters between two waypoints

def get_distance_meters(targetLocation,currentLocation):
	dLat=targetLocation.lat - currentLocation.lat
	dLon=targetLocation.lon - currentLocation.lon

	return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5

####Function to navigate to a location given in GPS coordinates

def goto(targetLocation):
	distanceToTargetLocation = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)

	vehicle.simple_goto(targetLocation)

	while vehicle.mode.name=="GUIDED":
		currentDistance = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)
		if currentDistance<distanceToTargetLocation*.01:
			print("Reached target waypoint.")
			time.sleep(2)
			break
		time.sleep(1)
	return None
    
##########MAIN EXECUTABLE###########

arm_and_takeoff(10)
wp1 = LocationGlobalRelative(40.655387,-73.970278,10)
goto(wp1)
end_loc1 = vehicle.location.global_relative_frame
print(wp1)
print(end_loc1)
print(get_distance_meters(wp1,end_loc1))
vehicle.close()
