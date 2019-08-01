#Offboard with tested funcitons

from dronekit import connect, Command, LocationGlobal,VehicleMode
from pymavlink import mavutil
import time, sys, argparse, math
import threading
"""
import cv2
import numpy as np
import picamera
"""
################################################################################################
# Settings
################################################################################################
x,y,z =0,0,0
exit_sp_buffer=False

#Calibrate the camera !!!
#Settings
gPerMeter=0 # No need to fix
gFrameCount=0 # No need to fix
startLandingAlt=0 # No need to fix
finishLandingAlt=0
MAX_ERROR_IN_DISTANCE = 1
MAX_ERROR_IN_ALTITUDE = 0.10 # THIS IS METER

horizontal_fov = 53.5
vertical_fov = 41.41
horizontal_resolution = 640
vertical_resolution = 480

################################################################################################
# CONNECTION
################################################################################################

connection_string       = '127.0.0.1:14540'

print ("Connecting")
vehicle = connect(connection_string, wait_ready=True)



################################################################################################
# FUNCTIONS FOR MOVING
################################################################################################
def setpoint_buffer():
    global x,y,z,exit_sp_buffer
    while True:
        if exit_sp_buffer:
            break
        msg=vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0,0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b111111111000,
            x,y,z,
            0,0,0,
            0,0,0,
            0,0)
        vehicle.send_mavlink(msg)
        vehicle.flush()
        time.sleep(0.3)
        
def tryArming():
    while True:
        vehicle._master.mav.command_long_send(
        1, # autopilot system id
        1, # autopilot component id
        400, # command id, ARM/DISARM
        0, # confirmation
        1, # arm!
        0,0,0,0,0,0 # unused parameters for this command
        )
        print("Trying arming")
        time.sleep(0.3)
        if (vehicle.armed == True):
            break
    print ("Armed :",vehicle.armed)


def startThread():
    t = threading.Thread(target=setpoint_buffer)

    t.daemon = True

    t.start()

def startOffboardMode():
    while vehicle.mode != "OFFBOARD":
        vehicle._master.set_mode_px4('OFFBOARD',None,None)
        print ("Trying offboard")
        time.sleep(0.3)

#New x y z => x + = new right
def goForXYZ(byX,byY,byZ):# Use byZ positive for UP
    global x,y,z
    x += byX
    y += byY
    z += -byZ

################################################################################################
# FUNCTIONS DETECTION
################################################################################################
def bf_fixer(takeoff_img):
    """
    HARD_CUT_POINT = 50

    camera = picamera.PiCamera()
    camera.start_preview()
    time.sleep(0.6)
    camera.capture('current_img.png', format='png')
    camera.stop_preview()
    camera.close()

    img1 = cv2.imread(takeoff_img, cv2.IMREAD_GRAYSCALE)
    img2 = cv2.imread('current_img.png', cv2.IMREAD_GRAYSCALE)

    orb = cv2.ORB_create()
    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)

    # Brute Force Matching
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)
    # print("Length of matches: " + str(len(matches)))
    matches = sorted(matches, key=lambda x: x.distance)[:HARD_CUT_POINT]
    # print("Worst distance: " + str(matches[len(matches)-1].distance))

    diff_arr_x = []
    diff_arr_y = []
    for match in matches:
        img1_idx = match.queryIdx
        img2_idx = match.trainIdx
        x1, y1 = kp1[img1_idx].pt
        x2, y2 = kp2[img2_idx].pt
        diff_arr_x.append(x1-x2)
        diff_arr_y.append(y1-y2)

    mean_x = statistics.mean(diff_arr_x)
    mean_y = statistics.mean(diff_arr_y)
    return (mean_x, mean_y)
    """
    return 350,350


def imageMassCoordinates(string_for_frame,altitude):
    (pxDistancex,pxDistancey)=bf_fixer(string_for_frame)
    distancex = altitude * math.tan(horizontal_fov* math.pi / 360) * 2
    distancey = altitude * math.tan(vertical_fov * math.pi / 360) * 2

    movex = pxDistancex * distancex / horizontal_resolution 
    movey = pxDistancey * distancey / vertical_resolution

    print("Move x :",movex)
    print("Move y :",movey)
    return movex,movey

def saveFrames(frameAltitude):#Saving PNG images at current altitude
    """
    camera = picamera.PiCamera()
    camera.start_preview()
    time.sleep(0.1)
    camera.capture('altitude_' + str(frameAltitude)+".png", format = 'png')
    camera.stop_preview()
    camera.close()
    """
    print("Frame captured altitude_"+str(frameAltitude)+".png")


################################################################################################
# MAIN
################################################################################################
tryArming()

startThread()

startOffboardMode()

saveFrames(0)
goForXYZ(0,0,1)
time.sleep(10)
saveFrames(1)
time.sleep(1)

goForXYZ(0,0,1)
time.sleep(10)
saveFrames(2)
time.sleep(1)

goForXYZ(0,0,1)
time.sleep(10)
saveFrames(3)
time.sleep(1)

goForXYZ(0,0,1)
time.sleep(10)
saveFrames(4)
time.sleep(1)

goForXYZ(0,0,1)
time.sleep(10)
saveFrames(5)
time.sleep(1)

goX,goY = imageMassCoordinates(5,5)
goForXYZ(goX,goY,0)
time.sleep(10)

goX,goY = imageMassCoordinates(4,5)
goForXYZ(goX,goY,-1)
time.sleep(10)

goX,goY = imageMassCoordinates(3,4)
goForXYZ(goX,goY,-1)
time.sleep(10)

goX,goY = imageMassCoordinates(2,3)
goForXYZ(goX,goY,-1)
time.sleep(10)

goX,goY = imageMassCoordinates(1,2)
goForXYZ(goX,goY,-1)
time.sleep(10)

goX,goY = imageMassCoordinates(0,1)
goForXYZ(goX,goY,-1)
time.sleep(10)

while vehicle.mode != "LAND":
    vehicle._master.set_mode_px4('LAND',None,None)
    print ("Trying land")
    time.sleep(0.3)

print ("DONE!")