#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import influxdb_client
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
import os
from datetime import datetime

class SensorDataListener:
    def __init__(self):
        # InfluxDB client setup
        self.token = os.getenv("INFLUXDB_TOKEN")
        if not self.token:
            raise ValueError("INFLUXDB_TOKEN environment variable not set")
            
        self.org = "magangbrin"
        self.url = "http://localhost:8086"
        self.bucket = "MEVi_sensor_data"
        
        # Initialize InfluxDB client
        try:
            self.client = InfluxDBClient(url=self.url, token=self.token, org=self.org)
            self.write_api = self.client.write_api(write_options=SYNCHRONOUS)
            rospy.loginfo("Successfully connected to InfluxDB")
        except Exception as e:
            rospy.logerr(f"Failed to connect to InfluxDB: {e}")
            raise

        # Log file setup
        self.log_file_path = "/home/sunnysmiles/Documents/ROSLOG/listenerlog_file.txt"
        
        # Initialize ROS node
        rospy.init_node('sensor_data_listener', anonymous=True)
        self.setup_subscribers()

    def setup_subscribers(self):
        # GPS subscribers
        rospy.Subscriber("gps_latitude", Float64, 
                        self.callback_wrapper("gps", "latitude", "gps_sensor"))
        rospy.Subscriber("gps_longitude", Float64, 
                        self.callback_wrapper("gps", "longitude", "gps_sensor"))

        # Compass subscriber
        rospy.Subscriber("compass_heading", Float64, 
                        self.callback_wrapper("compass", "heading", "compass_sensor"))

        # Ultrasonic subscribers
        rospy.Subscriber("ultrasonic_pressure", Float64, 
                        self.callback_wrapper("ultrasonic", "pressure", "ultra1"))
        rospy.Subscriber("ultrasonic_speed", Float64, 
                        self.callback_wrapper("ultrasonic", "speed", "ultra1"))
        rospy.Subscriber("ultrasonic2_pressure", Float64, 
                        self.callback_wrapper("ultrasonic", "pressure", "ultra2"))
        rospy.Subscriber("ultrasonic2_speed", Float64, 
                        self.callback_wrapper("ultrasonic", "speed", "ultra2"))

        # INA219 subscribers
        rospy.Subscriber("ina219_temperature", Float64, 
                        self.callback_wrapper("ina219", "temperature", "inabattery"))
        rospy.Subscriber("ina219_voltage", Float64, 
                        self.callback_wrapper("ina219", "voltage", "inasteer"))
        rospy.Subscriber("ina219_current", Float64, 
                        self.callback_wrapper("ina219", "current", "inasteer"))
        rospy.Subscriber("ina219_power", Float64, 
                        self.callback_wrapper("ina219", "power", "inasteer"))

        # DAC subscriber
        rospy.Subscriber("dac_velocity", Float64, 
                        self.callback_wrapper("dac", "velocity", "dac_sensor"))

        # Absolute encoder subscribers
        rospy.Subscriber("abs_encoder_wheel_angle", Float64, 
                        self.callback_wrapper("abs_encoder", "wheel_angle", "left_encoder"))
        rospy.Subscriber("abs_encoder_current_angle", Float64, 
                        self.callback_wrapper("abs_encoder", "current_angle", "right_encoder"))

        # Incremental encoder subscribers
        rospy.Subscriber("inc_encoder_velocity", Float64, 
                        self.callback_wrapper("inc_encoder", "velocity", "left_encoder"))
        rospy.Subscriber("inc_encoder_ticks", Float64, 
                        self.callback_wrapper("inc_encoder", "ticks", "right_encoder"))
                        
	# ACS758 subscribers
        rospy.Subscriber("acs758_current", Float64, 
                        self.callback_wrapper("acs758", "current", "acs758steer"))
			
    def callback_wrapper(self, sensor_type, measurement, sensor_id):
        def callback(data):
            try:
                # Create point with standardized tags for all measurements
                point = Point(f"{sensor_type}_measurements")
                point.tag("sensor_type", sensor_type)
                point.tag("measurement_type", measurement)
                point.tag("sensor_id", sensor_id)

                # Add value and timestamp
                point.field("value", float(data.data))
                point.time(datetime.utcnow(), WritePrecision.NS)

                # Write to InfluxDB
                self.write_api.write(bucket=self.bucket, org=self.org, record=point)
                
                # Log to console
                log_msg = f"Received {sensor_type} {measurement} from {sensor_id}: {data.data}"
                rospy.loginfo(log_msg)
                
                # Log to file
                self.write_to_log_file(sensor_type, measurement, data.data, sensor_id)
                
            except Exception as e:
                rospy.logerr(f"Error processing {sensor_type} {measurement}: {e}")

        return callback

    def write_to_log_file(self, sensor_type, measurement, value, sensor_id):
        try:
            with open(self.log_file_path, 'a') as log_file:
                timestamp = rospy.get_time()
                log_entry = f"{timestamp},{sensor_type},{measurement},{sensor_id},{value}\n"
                log_file.write(log_entry)
        except Exception as e:
            rospy.logerr(f"Error writing to log file: {e}")

    def run(self):
        try:
            rospy.loginfo("Sensor data listener is running...")
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("Shutting down sensor data listener...")
        finally:
            if hasattr(self, 'client'):
                self.client.close()

if __name__ == '__main__':
    try:
        listener = SensorDataListener()
        listener.run()
    except Exception as e:
        rospy.logerr(f"Failed to start listener: {e}")