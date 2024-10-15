#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import influxdb_client, os
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS

# InfluxDB client setup
token = os.getenv("INFLUXDB_TOKEN") # Export your own influxdb token variable to your environment
org = "<your_influxdb_org>"
url = "http://<your_ip>:8086" # or access via localhost:8086
bucket = "<your_influxdb_bucket>"
client = InfluxDBClient(url=url, token=token, org=org)
write_api = client.write_api(write_options=SYNCHRONOUS)

def write_to_influxdb(lat, lon, heading):
    # Get the ROS time (as float)
    ros_time = rospy.get_time()

    # Convert ROS time to integer seconds for WritePrecision.S
    timestamp_in_seconds = int(ros_time)  # Converting the float timestamp to integer seconds

    # GPS point
    gps_point = (
        Point("gps_dummydata")
        .tag("sensor_id", "gps")
        .field("latitude", lat)
        .field("longitude", lon)
        .time(timestamp_in_seconds, WritePrecision.S)  # Corrected to use integer seconds
    )

    # Compass point
    compass_point = (
        Point("compass_dummydata")
        .tag("sensor_id", "compass")
        .field("heading", heading)
        .time(timestamp_in_seconds, WritePrecision.S)  # Corrected to use integer seconds
    )

    # Write to InfluxDB
    write_api.write(bucket=bucket, org=org, record=gps_point)
    write_api.write(bucket=bucket, org=org, record=compass_point)

def gps_latitude_callback(data):
    rospy.loginfo(f"Received GPS Latitude: {data.data}")
    global gps_lat
    gps_lat = data.data

def gps_longitude_callback(data):
    rospy.loginfo(f"Received GPS Longitude: {data.data}")
    global gps_lon
    gps_lon = data.data

def compass_callback(data):
    rospy.loginfo(f"Received Compass Heading: {data.data}")
    global compass_heading
    compass_heading = data.data
    write_to_influxdb(gps_lat, gps_lon, compass_heading)

def listener():
    rospy.init_node('gps_compass_listener', anonymous=True)
    
    rospy.Subscriber("gps_latitude", Float64, gps_latitude_callback)
    rospy.Subscriber("gps_longitude", Float64, gps_longitude_callback)
    rospy.Subscriber("compass_heading", Float64, compass_callback)
    
    rospy.spin()

if __name__ == '__main__':
    global gps_lat, gps_lon, compass_heading
    gps_lat, gps_lon, compass_heading = 0.0, 0.0, 0.0
    listener()