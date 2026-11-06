# TMSerial
This package implements a ROS node of TransducerM AHRS/IMU from SYD Dynamics. 

# Messages
This package publishes message types:
- sensor_msgs/Imu :            The standard Imu message from ROS
- sensor_msgs/MagneticField:   The standard Imu mag message from ROS

# Topics
This package publishes three topics:
-TransducerM_pub_imu : Standard ROS imu data package, including quaternion data, acceleration data, and gyroscope data
-TransducerM_pub_mag : Standard ROS magneticfield data package
-TransducerM_pub_rpy : This topic shares the same data structure as the standard ROS imu_data_mag topic, while it is used for publishing Roll, Pitch and Yaw output from TransducerM. The definition of data fields shown as below:
                imu_data_rpy.magnetic_field.x =  roll                    
                imu_data_rpy.magnetic_field.y =  pitch
                imu_data_rpy.magnetic_field.z =  yaw
                
# How to use it
1. Create a work folder:
       $mkdir -p ~/imu_ws/src
2. Copy all source code of this ROS package into the src folder created above
3. Setup parameters in ./src/TMSerial.cpp
       For example: change imu_port value to your actual serial port name and setup baudrate
       DEVICE_PORT is USB prot name
       BAUD_RATE   is baudrate
4. Compile 
       $cd ~/imu_ws
           Then if you run $pwd the path should be similar as below:
           /home/your_user_name/imu_ws
       $catkin_make
       $cd ~
       $source ~/imu_ws/devel/setup.sh 
       $roscore
       $rosrun TransducerM_pkg TMSerial
5. View topics
       $rostopic list 
       $rostopic echo /TransducerM_xxxx

If you find any issue please leave a message at [our website](https://www.syd-dynamics.com/contact-us/)

Last updated on Aug 1, 2024




