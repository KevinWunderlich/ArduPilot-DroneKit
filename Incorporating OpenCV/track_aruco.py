#!/usr/bin/env python

#library imports for use with this script
import rospy
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import sys
import time
import math
import numpy as np
import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
from array import array

##############
##Variables###
##############

id_to_find = 129 #aruco_id that the script will be looking for
marker_size = 40 #CM

#specifies which aruco dictionary the marker will be pulled from
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
#creates the parameters for the aruco detection process
parameters = aruco.DetectorParameters_create()

#camera resolution
horizontal_res = 640
vertical_res = 480

#Camera fields of view
horizontal_fov = 62.2 * (math.pi / 180) #62.2 for picam V2, converted into radians
vertical_fov = 48.8 * (math.pi / 180) #48.8 for V2, again converted into radians

#counters to iterate depending on whether the markers were found or not
found_count=0
notfound_count=0

#camera intrinsics, details of how to find this for the Gazebo camera are included in the report
dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_matrix = [[530.8269276712998, 0.0, 320.5],[0.0, 530.8269276712998, 240.5],[0.0, 0.0, 1.0]]
#convert both of these arrays into numpy arrays for OpenCV processing
np_camera_matrix = np.array(camera_matrix)
np_dist_coeff = np.array(dist_coeff)

#variables to simulate a more accurate real-world fps rate
time_last=0
time_to_wait = .1 ##100 ms

####Functions####

#creates a new ROS publisher called '/camera/color/image_new' of type 'image'
newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)

#function to process every new message recieved by the virtual camera feed
#will be included in the subscriber function as the callback function
def msg_receiver(message):
    global notfound_count, found_count, time_last, time_to_wait, id_to_find

    height=message.height
    width=message.width

    #logic to create a more accurate fps rate
    if time.time() - time_last > time_to_wait:
        np_data = rnp.numpify(message) #Deserialize image data into numpy array
        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY) #convert image into gray image

        ids = ''
        #OpenCV detect markers function, returns the 2d position of the corners of the marker detected as well as it's id
        (corners, ids, rejected) = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)

        #if a marker was detected and it's id matches the desired id, the following is ran
        try:
            if ids is not None:
                if ids[0]==id_to_find:
                    #estimates the pose of the marker with respect to the camera
                    #output (stored in ret) will contain a rotation vector and a translation / position vector
                    ret = aruco.estimatePoseSingleMarkers(corners,marker_size,cameraMatrix=np_camera_matrix,distCoeffs=np_dist_coeff)

                    #rvec is the rotation vector, stored at index 0 in the output of the above function
                    #tvec is basically functioning as the distance the camera is from the marker
                    (rvec, tvec) = (ret[0][0,0,:],ret[1][0,0,:])
                    x = '{:.2f}'.format(tvec[0]) # X axis error/distance between camera and aruco in CM
                    y = '{:.2f}'.format(tvec[1]) # Y axis error/distance between camera and aruco in CM
                    z = '{:.2f}'.format(tvec[2]) # Z axis error/distance between camera and aruco in CM

                    marker_position = 'MARKER POSITION: x='+x+' y='+y+' z='+z

                    #modifies the image fed into the function, draws a border around the detected marker
                    aruco.drawDetectedMarkers(np_data,corners)
                    #draws the x, y and z axes of the marker in relation to the camera
                    aruco.drawAxis(np_data,np_camera_matrix,np_dist_coeff,rvec,tvec,10)
                    ##putText(image, text_to_draw,position,font_face,fontScale,color,thickness)
                    cv2.putText(np_data,marker_position,(10,50),0,.7,(255,0,0),thickness=2)
                    print(marker_position)
                    print('FOUND COUNT: '+str(found_count)+ ' NOTFOUND COUNT: '+str(notfound_count))

                    found_count = float(found_count + 1)
                else:
                    notfound_count= float(notfound_count+1)
            else:
                notfound_count=float(notfound_count+1)
            total_count = float(notfound_count+found_count)
            percent_found = float(found_count/total_count)*100

            print('Percent of frames that found aruco id: '+str(percent_found))
        except Exception as e:
            print('Target likely not found')
            print(e)
            notfound_count=notfound_count+1
        
        #convert the processed camera image into a Ros message
        new_msg = rnp.msgify(Image, np_data,encoding='rgb8')
        #have the 'newimg_pub' publisher node publish the processed image
        newimg_pub.publish(new_msg)
        time_last = time.time()
    else:
        return None

#function to subscribe to the ROS node
def subscriber():
    rospy.init_node('drone_node',anonymous=False)
    sub = rospy.Subscriber('/camera/color/image_raw', Image, msg_receiver)
    rospy.spin()


if __name__=='__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
