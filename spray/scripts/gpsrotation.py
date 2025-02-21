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

# Define the GPS coordinates to visit
gps_coordinates = [
    (33.6424731, 72.9911732),
    (33.6424904, 72.9910280)
]


def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculate the great-circle distance between two points
    on the Earth's surface using the Haversine formula.
    """
    # Radius of Earth in meters
    R = 6378137.0

    # Convert latitude and longitude from degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    # Differences in coordinates
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    # Haversine formula
    a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Distance in meters
    distance = R * c
    return distance


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

        # Perform 360-degree rotations about the yaw axis 5-6 times
        rospy.loginfo("Performing rotations at target GPS coordinates...")
        for _ in range(2):  # Rotate 2 times
            for yaw in range(0, 360, 30):  # Change yaw angle in 30-degree increments
                drone.goto_location_heading(latitude, longitude, altitude=2, yaw=math.radians(yaw))
                rospy.sleep(0.5)  # Wait to complete the yaw adjustment
        rospy.loginfo("Rotation sequence completed.")

    if drone.set_mode(mode="LAND"):
        rospy.loginfo("Drone is landing...")
        break
    else:
        rospy.logerr("Drone failed to land. Aborting Mission...")
        sys.exit(1)

# Disarm the drone
rospy.loginfo("Disarming the drone...")
while not drone.check_land_complete():
    # print("...")
   time.sleep(0.1)
    
time.sleep(2)
    
rospy.loginfo("drone has successfully landed.")
    
rospy.loginfo("Mission Successfull.")
