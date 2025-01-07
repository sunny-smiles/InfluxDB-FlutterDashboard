#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import time

def talker():
    rospy.init_node('sensor_data_talker', anonymous=True)
    
    # Define publishers for all the sensors
    gps_lat_pub = rospy.Publisher('gps_latitude', Float64, queue_size=10)
    gps_lon_pub = rospy.Publisher('gps_longitude', Float64, queue_size=10)
    compass_pub = rospy.Publisher('compass_heading', Float64, queue_size=10)

    ultrasonic_pressure_pub = rospy.Publisher('ultrasonic_pressure', Float64, queue_size=10)
    ultrasonic_speed_pub = rospy.Publisher('ultrasonic_speed', Float64, queue_size=10)
    
    ultrasonic2_pressure_pub = rospy.Publisher('ultrasonic2_pressure', Float64, queue_size=10)
    ultrasonic2_speed_pub = rospy.Publisher('ultrasonic2_speed', Float64, queue_size=10)

    ina219_temp_pub = rospy.Publisher('ina219_temperature', Float64, queue_size=10)
    ina219_voltage_pub = rospy.Publisher('ina219_voltage', Float64, queue_size=10)
    ina219_current_pub = rospy.Publisher('ina219_current', Float64, queue_size=10)
    ina219_power_pub = rospy.Publisher('ina219_power', Float64, queue_size=10)

    dac_velocity_pub = rospy.Publisher('dac_velocity', Float64, queue_size=10)

    abs_encoder_wheel_angle_pub = rospy.Publisher('abs_encoder_wheel_angle', Float64, queue_size=10)
    abs_encoder_current_angle_pub = rospy.Publisher('abs_encoder_current_angle', Float64, queue_size=10)

    inc_encoder_velocity_pub = rospy.Publisher('inc_encoder_velocity', Float64, queue_size=10)
    inc_encoder_ticks_pub = rospy.Publisher('inc_encoder_ticks', Float64, queue_size=10)

    acs758_current_pub = rospy.Publisher('acs758_current', Float64, queue_size=10)
    
    counter = 0
    rate = rospy.Rate(1)  # Publish every second
    
    # Open a log file for saving
    log_file_path = "/home/sunnysmiles/Documents/ROSLOG/talkerlog_file.txt"
    with open(log_file_path, 'a') as log_file:
        while not rospy.is_shutdown():
            # Simulate GPS, compass, ultrasonic, INA219, DAC, and encoder data
            latitude = 12.9716 + (0.0001 * rospy.get_time() % 1)
            longitude = 77.5946 + (0.0001 * rospy.get_time() % 1)
            heading = (rospy.get_time() % 360)

            pressure = 100 + (0.5 * rospy.get_time() % 10)  # Dummy pressure data in Pascals
            speed = 50 + (2.0 * rospy.get_time() % 20)  # Dummy speed data in m/s
            
            pressure2 = 100 + (0.7 * rospy.get_time() % 10)  # Different variation for second sensor
            speed2 = 50 + (2.5 * rospy.get_time() % 20)     # Different variation for second sensor      
            
            # Temperature alternating between 35°C and 29°C every 15 seconds
            if (counter // 15) % 2 == 0:  # First 15 seconds
                temp = 35.0
            else:  # Next 15 seconds
                temp = 29.0
            
            counter += 1
                  
            voltage = 12.0 + (0.05 * rospy.get_time() % 2)  # Dummy voltage in volts
            current = 1.5 + (0.1 * rospy.get_time() % 3)  # Dummy current in Amps
            power = voltage * current  # Power = Voltage * Current

            dac_velocity = 100 + (3 * rospy.get_time() % 10)  # Dummy velocity data for DAC

            wheel_angle = (rospy.get_time() % 360)  # Dummy absolute encoder wheel angle
            current_angle = wheel_angle  # Assume current angle is same as wheel angle

            inc_velocity = 5 + (0.1 * rospy.get_time() % 10)  # Dummy incremental encoder velocity
            ticks = rospy.get_time() % 1000  # Dummy tick count

            acs_current = 2.0 + (0.2 * rospy.get_time() % 5)  # Dummy current in Amps
            
            current_time = rospy.get_time()
            
            # Log to terminal
            rospy.loginfo(f"Publishing at time {current_time}: GPS({latitude}, {longitude}), Compass({heading}), "
                          f"Ultrasonic(Pressure: {pressure}, Speed: {speed}), "
                          f"Ultrasonic2(Pressure: {pressure2}, Speed: {speed2}), "
                          f"INA219(Temp: {temp}°C, Voltage: {voltage}, Current: {current}, Power: {power}), "
                          f"DAC Velocity: {dac_velocity}, Absolute Encoder(Wheel Angle: {wheel_angle}, Current Angle: {current_angle}), "
                          f"Incremental Encoder(Velocity: {inc_velocity}, Ticks: {ticks}), "
                          f"ACS758(Current: {acs_current})")
            
            # Log to file
            log_file.write(f"{current_time}, {latitude}, {longitude}, {heading}, {pressure}, {speed}, "
                           f"{pressure2}, {speed2}, "
                           f"{temp}, {voltage}, {current}, {power}, {dac_velocity}, {wheel_angle}, "
                           f"{current_angle}, {inc_velocity}, {ticks}, "
                           f"{acs_current}\n")
            
            # Publish to ROS topics
            gps_lat_pub.publish(latitude)
            gps_lon_pub.publish(longitude)
            compass_pub.publish(heading)

            ultrasonic_pressure_pub.publish(pressure)
            ultrasonic_speed_pub.publish(speed)
            
            ultrasonic2_pressure_pub.publish(pressure2)
            ultrasonic2_speed_pub.publish(speed2)

            ina219_temp_pub.publish(temp)
            ina219_voltage_pub.publish(voltage)
            ina219_current_pub.publish(current)
            ina219_power_pub.publish(power)

            dac_velocity_pub.publish(dac_velocity)

            abs_encoder_wheel_angle_pub.publish(wheel_angle)
            abs_encoder_current_angle_pub.publish(current_angle)

            inc_encoder_velocity_pub.publish(inc_velocity)
            inc_encoder_ticks_pub.publish(ticks)
            
            acs758_current_pub.publish(acs_current)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass