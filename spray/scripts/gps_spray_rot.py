#!/usr/bin/python3

from Spray_Drone_API import *
import rospy
import time
import math
from mavros_msgs.srv import CommandLong

def send_servo_command(servo_num, pwm_value):
    """Function to send a command to control a servo via MAVROS."""
    rospy.wait_for_service('/drone1/mavros/cmd/command')
    try:
        command_service = rospy.ServiceProxy('/drone1/mavros/cmd/command', CommandLong)
        response = command_service(
            command=183,  # MAV_CMD_DO_SET_SERVO
            param1=servo_num,  # Servo number
            param2=pwm_value,  # PWM value
            param3=0, param4=0, param5=0, param6=0, param7=0
        )
        if response.success:
            rospy.loginfo(f"Servo {servo_num} set to PWM {pwm_value}")
        else:
            rospy.logerr("Failed to send servo command")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


# Initialize Drone
drone = SprayDrone(name='drone1')
rospy.loginfo("Waiting for drone to switch to Guided Mode")
drone.wait_for_guided()
drone.set_stream_rate()
rospy.loginfo("Drone's Mode is now GUIDED.")
drone.wait_for_GPS_Fix()

# Arm Drone
rospy.loginfo("Arming Drone")
drone_response = drone.arm()

if drone_response.success:
    rospy.loginfo("Drone's Mode is now ARMED.")
else:
    rospy.logerr("Drone is not Armable. Aborting Mission...")
    sys.exit(1)

# Takeoff
drone_min_altitude = 2
drone_response = drone.takeoff(altitude=drone_min_altitude)

if drone_response.success:
    rospy.loginfo("Drone is Taking off.")
else:
    rospy.logerr("Drone failed to takeoff. Aborting Mission...")
    sys.exit(1)

while not drone.check_takeoff_complete():
    time.sleep(0.1)

# Define GPS coordinates to visit
gps_coordinates = [
    (33.6424731, 72.9911732),
    (33.6424904, 72.9910280)
]


def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate the great-circle distance between two GPS coordinates using the Haversine formula."""
    R = 6378137.0  # Radius of Earth in meters

    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c  # Distance in meters


# Main execution loop
while not rospy.is_shutdown():
    for latitude, longitude in gps_coordinates:
        rospy.loginfo(f"Moving to GPS coordinate: Latitude: {latitude}, Longitude: {longitude}, Altitude: 5m")
        drone.goto_location(latitude, longitude, altitude=3)

        # Check if the drone has reached the target GPS coordinates
        rospy.loginfo("Waiting to reach target GPS coordinates...")
        while True:
            current_lat = drone.data.global_position.latitude
            current_lon = drone.data.global_position.longitude
            distance = haversine_distance(current_lat, current_lon, latitude, longitude)

            rospy.loginfo(f"Current position: Lat: {current_lat}, Lon: {current_lon}, Distance to target: {distance:.2f} meters")

            if distance < 2:  # Threshold distance to consider target reached
                rospy.loginfo("Reached target GPS coordinates.")
                break
            rospy.sleep(1)

        # **Activate the servo first**
        rospy.loginfo("Activating servo before rotation...")
        send_servo_command(5, 2000)  # Move Servo 5 to PWM 2000 (Turn on)
        rospy.sleep(2)  # Wait for servo to complete action
        rospy.loginfo("Servo action completed.")

        # **Perform 360-degree rotations after activating the servo**
        rospy.loginfo("Performing rotations at target GPS coordinates...")
        for _ in range(2):  # Rotate 2 times
            for yaw in range(0, 360, 30):  # Change yaw angle in 30-degree increments
                drone.goto_location_heading(latitude, longitude, altitude=2, yaw=math.radians(yaw))
                rospy.sleep(0.5)  # Wait to complete the yaw adjustment
        rospy.loginfo("Rotation sequence completed.")

        # **Deactivate the servo after rotations**
        rospy.loginfo("Deactivating servo after rotation...")
        send_servo_command(5, 1000)  # Move Servo 5 to PWM 1000 (Turn off)
        rospy.sleep(2)  # Wait for servo to complete action
        rospy.loginfo("Servo deactivation completed.")

    if drone.set_mode(mode="LAND"):
        rospy.loginfo("Drone is landing...")
        break
    else:
        rospy.logerr("Drone failed to land. Aborting Mission...")
        sys.exit(1)

# Disarm the drone after landing
rospy.loginfo("Disarming the drone...")
while not drone.check_land_complete():
    time.sleep(0.1)
    
time.sleep(2)

rospy.loginfo("Drone has successfully landed.")
rospy.loginfo("Mission Successful.")

