#!/usr/bin/python3

from Spray_Drone_API import *
import rospy
import time

drone = SprayDrone(name='drone1')
rospy.loginfo("Waiting for drone to switch to Guided Mode")
drone.wait_for_guided()
drone.set_stream_rate()
rospy.loginfo("Drone's Mode is now GUIDED.")
drone.wait_for_GPS_Fix() 

rospy.loginfo("Arming Drone")
drone_response = drone.arm()

if drone_response.success:
    rospy.loginfo("Drone's Mode is now ARMED.")
else:
    rospy.logerr("Drone is not Armable. Aborting Mission...")
    sys.exit(1)

drone_min_altitude =2     
drone_response = drone.takeoff(altitude=drone_min_altitude)

if drone_response.success:
    rospy.loginfo("Drone is Taking off.")
else:
    rospy.logerr("Drone failed to takeoff. Aborting Mission...")
    sys.exit(1)

while not drone.check_takeoff_complete():
    # Waiting for leader to complete takeoff
    time.sleep(0.1)

# Hover for few seconds
rospy.loginfo("Takeoff Complete. Hovering...")
time.sleep(3)

#rospy.loginfo("Sending custom PWM signal to main pin 8...")
#drone.set_pwm(channel=5, pwm_value=1800)

rospy.loginfo("Landing Drone...")
drone.set_mode(mode="LAND")
    
while not drone.check_land_complete():
    # print("...")
   time.sleep(0.1)
    
time.sleep(2)
    
rospy.loginfo("drone has successfully landed.")
    
rospy.loginfo("Mission Successfull.")
