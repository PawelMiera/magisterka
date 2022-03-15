import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import *
import time


def setOffboardMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='OFFBOARD')  # return true or false
    except rospy.ServiceException, e:
        print "Service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e

def setArm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException, e:
        print "Service arm call failed: %s" % e


def setDisarm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(False)
    except rospy.ServiceException, e:
        print "Service arm call failed: %s" % e


def setTakeoffMode(altitude):
    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
        takeoffService(altitude=altitude, latitude=0, longitude=0, min_pitch=0, yaw=0)
    except rospy.ServiceException, e:
        print "Service takeoff call failed: %s" % e


def armAndTakeoff(altitude):
    setArm()
    time.sleep(0.5)
    setTakeoffMode(altitude)

def setLandMode():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        # http://wiki.ros.org/mavros/CustomModes for custom modes
        isLanding = landService(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
    except rospy.ServiceException, e:
        print "service land call failed: %s. The vehicle cannot land " % e


def menu():
    print "Press"
    print "1: to ARM"
    print "2: to DISARM"
    print "3: to set mode to OFFBOARD"
    print "4: to ARM and TAKEOFF"
    print "5: to set mode to LAND"


def myLoop():
    x = '1'
    while ((not rospy.is_shutdown()) and (x in ['1', '2', '3', '4', '5', '6', '7'])):
        menu()
        x = raw_input("Enter your input: ");
        if (x == '1'):
            setArm()
        elif (x == '2'):
            setDisarm()
        elif (x == '3'):
            setOffboardMode()
        elif (x == '4'):
            armAndTakeoff(11)
        elif (x == '5'):
            setLandMode()
        else:
            print "Exit"


if __name__ == '__main__':
    myLoop()