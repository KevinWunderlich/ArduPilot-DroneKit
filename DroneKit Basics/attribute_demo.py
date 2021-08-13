##########DEPENDENCIES#############

from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
import time
import socket
import exceptions
import math
import argparse

##########CONNECTION STRING###########

vehicle = connect('tcp:127.0.0.1:5763',wait_ready=True)

#Version and attributesggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggg
vehicle.wait_ready('autopilot_version')
print('Autopilot version: %s'%vehicle.version)

#Does the firmware support the companion pc to set the attitude
print('Supports set attitude from companion: %s'%vehicle.capabilities.set_attitude_target_local_ned)

#Read actual position
print('Position: %s'%vehicle.location.global_relative_frame)

#Read the actual attitude roll, pitch, yaw
print('Attitude: %s'%vehicle.attitude)

#Read the actual velocity (m/s)
print('Velocity: %s'%vehicle.velocity) #NED: North East Down convention

#When did we receive last heartbeat
print('Last Heartbeat: %s'%vehicle.last_heartbeat)

#Is the vehicle good to arm
print('Is the vehicle armable: %s'%vehicle.is_armable)

#What is total groundspeed
print('Groundspeed: %s'%vehicle.groundspeed) #This is settable

#What is the actual flight mode
print('Mode: %s'%vehicle.mode.name) 	     #This is settable

#Is the vehicle armed
print('Armed: %s'%vehicle.armed) 	     #This is settable

#Is state estimation filter ok
print('EKF Ok: %s'%vehicle.ekf_ok)

vehicle.close()
