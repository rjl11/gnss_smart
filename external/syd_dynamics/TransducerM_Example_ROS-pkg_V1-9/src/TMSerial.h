#ifndef TMSERIAL_H
#define TMSERIAL_H

//linux serial lib:
#include <stdio.h>
#include "serialib/serialib.h"

// To use the communication library, we need to include the following
// two header files:
#include "EasyProfile/EasyObjectDictionary.h"
#include "EasyProfile/EasyProfile.h"

//ros message header file
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

// Debug switch:
//#define  DEBUG_MODE
// Enable IMU mag msg
//#define  ROS_MAG_MSG
// Enable IMU RPY msg
//#define  ROS_RPY_MSG

class TMSerial
{
public:
    TMSerial();
    ~TMSerial();

private:
    serialib* serialib1; //we use linux serialib to interface with serial port.

public:
    char Serialport_Open();
    bool On_SerialRX();  // RX (Receive Sensor Data)
    sensor_msgs::Imu imu_data_msg;

    #ifdef ROS_MAG_MSG
    sensor_msgs::MagneticField imu_data_mag;
    #endif

    #ifdef ROS_RPY_MSG
    sensor_msgs::MagneticField imu_data_rpy;
    #endif
};

#endif // TMSERIAL_H
