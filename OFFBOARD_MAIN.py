#Offboard with tested funcitons

from dronekit import connect, Command, LocationGlobal,VehicleMode
from pymavlink import mavutil
import time, sys, argparse, math
import threading
"""
from statistics import mean
import cv2
import numpy as np
import picamera
"""
################################################################################################
# Settings
################################################################################################
x,y,z =0,0,0


#Calibrate the camera !!!
#Settings
gPerMeter=0 # No need to fix
gFrameCount=0 # No need to fix
startLandingAlt=0 # No need to fix
finishLandingAlt=0
MAX_ERROR_IN_DISTANCE = 1
MAX_ERROR_IN_ALTITUDE = 0.10 # THIS IS METER
MAX_ERROR_IN_DEGREES = 0.8
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
    global x,y,z
    while True:
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
def checkAltitude(tagretAltitude):
    count = 0
    while True:
        if (vehicle.location.global_relative_frame.alt < (tagretAltitude + MAX_ERROR_IN_ALTITUDE)):
            if(vehicle.location.global_relative_frame.alt > (tagretAltitude - MAX_ERROR_IN_ALTITUDE)):
                count += 1
                if(count >= 10):
                    print("Target altitude reached")
                    break
            else:
                count = 0
        else:
            count = 0
        time.sleep(0.5)

def bf_fixer(takeoff_img):
    """
    camera = picamera.PiCamera()
    camera.start_preview()
    """
    time.sleep(1.5) # allow the autoexposure function to work
    const = 180 / math.pi
    pitch,roll = 100, 100
    while not (abs(roll) + abs(pitch))<MAX_ERROR_IN_DEGREES:
        pitch = vehicle.attitude.pitch * const # pitch in degrees
        roll = vehicle.attitude.roll * const
        print("Trying to stabilize for taking frame")
        time.sleep(0.3)
    """
    HARD_CUT_POINT = 50

    
    #time.sleep(0.6)
    current_img = "current_" + str(vehicle.locaiton.global_relative_frame.alt)+".png"
    camera.capture(current_img, format='png')
    camera.stop_preview()
    camera.close()

    img1 = cv2.imread(takeoff_img, cv2.IMREAD_GRAYSCALE)
    img2 = cv2.imread(current_img, cv2.IMREAD_GRAYSCALE)

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
    
    string = "altitude_" + str(string_for_frame) + ".png" #path to saveFrame
    
    (pxDistancex,pxDistancey)=bf_fixer(string)
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
    """
    time.sleep(1.5) # allow the autoexposure function to work
    const = 180 / math.pi
    pitch,roll = 1, 1
    while not (abs(roll) + abs(pitch))<MAX_ERROR_IN_DEGREES:
        pitch = vehicle.attitude.pitch * const # pitch in degrees
        roll = vehicle.attitude.roll * const
        print("Trying to stabilize for taking frame")
        time.sleep(0.2)
    """
    
    #time.sleep(0.1)
    camera.capture('altitude_' + str(frameAltitude)+".png", format = 'png')
    camera.stop_preview()
    camera.close()
    """
    print("Frame captured altitude_"+str(frameAltitude)+".png")

def set_home_location():
    while not vehicle.home_location==vehicle.location.global_frame:
        print("Setting home position...")
        vehicle.home_location=vehicle.location.global_frame
        time.sleep(1)

home_position_set = False
#Create a message listener for home position fix
@vehicle.on_message('HOME_POSITION')
def listener(self, name, home_position):
    global home_position_set
    home_position_set = True

def print_status():
    # Display basic vehicle state
    print (" Type: %s" % vehicle._vehicle_type)
    print (" Armed: %s" % vehicle.armed)
    print (" System status: %s" % vehicle.system_status.state)
    print (" GPS: %s" % vehicle.gps_0)
    print (" Alt: %s" % vehicle.location.global_relative_frame.alt)


def missionController(start,finish):

    while not home_position_set:
        print ("Waiting for home position...")
        time.sleep(1)

    print_status()

    tryArming()
    startThread()
    startOffboardMode()

    print_status()

    for i in range(finish-start):
        saveFrames(i)
        time.sleep(1)
        goForXYZ(0,0,1)
        checkAltitude(i+1)
    
    print("Landing started!")
    #goX,goY = imageMassCoordinates(finish,finish)
    #goForXYZ(goX,goY,0)
    for i in range(finish,start,-1):
        
        print("Looking picture altitude",i-1)
        print("Altitude :",i)
        print("Real altitude:",vehicle.location.global_relative_frame.alt)
        print("Target altitude :",i-1)
        goX,goY = imageMassCoordinates(i-1,i)
        goForXYZ(goX,goY,-1)
        checkAltitude(i-1)
    print("Land !")
    while vehicle.mode != "LAND":
        vehicle._master.set_mode_px4('LAND',None,None)
        print ("Trying land")
        time.sleep(0.3)
    time.sleep(2)
    print ("Landed!")

    
    vehicle.close()
        

################################################################################################
# MAIN
################################################################################################
print("Starting Mission...")
time.sleep(1)
missionController(0,5)
time.sleep(1)
print("DONE !")
