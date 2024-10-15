#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
import time

def talker():
    rospy.init_node('gps_compass_talker', anonymous=True)
    
    gps_lat_pub = rospy.Publisher('gps_latitude', Float64, queue_size=10)
    gps_lon_pub = rospy.Publisher('gps_longitude', Float64, queue_size=10)
    compass_pub = rospy.Publisher('compass_heading', Float64, queue_size=10)

    rate = rospy.Rate(1)  # Publish every second
    
    while not rospy.is_shutdown():
        latitude = 12.9716 + (0.0001 * time.time() % 1)
        longitude = 77.5946 + (0.0001 * time.time() % 1)
        heading = (time.time() % 360)
        
        rospy.loginfo(f"Publishing GPS: {latitude}, {longitude}, Compass: {heading}")
        
        gps_lat_pub.publish(latitude)
        gps_lon_pub.publish(longitude)
        compass_pub.publish(heading)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass