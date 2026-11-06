#include "ros/ros.h"
#include "TMSerial.h"

// TransducerM Interface:
EasyObjectDictionary eOD;
EasyProfile eP(&eOD);

// Define the serial port here:
#define DEVICE_PORT "/dev/ttyUSB0"
#define BAUD_RATE (115200)
TMSerial::TMSerial()
{
    serialib1 = new serialib;
}

TMSerial::~TMSerial()
{
}

char TMSerial::Serialport_Open()
{
    int Ret;
    Ret = serialib1->openDevice(DEVICE_PORT, BAUD_RATE); // Open serial link at 115200 bauds
    if (Ret != 1)
    {                                                               // If an error occured...
        printf("Error while opening port. Permission problem ?\n"); // ... display a message ...
        return Ret;                                                 // ... quit the application
    }
    printf("Serial port opened successfully !\n");
    return 1;
}

//----------------------------------------------------------------------------------------
// Serial Receive Data Example
bool TMSerial::On_SerialRX()
{
    char serialBuffer[256];
    int ret = serialib1->readBytes(serialBuffer, sizeof(serialBuffer), 1, 100); // Step 1: read the received data buffer of the Serial Port
#ifdef DEBUG_MODE
    ROS_INFO("rxsize = %d", ret);
#endif
    if (ret <= 0)
    {
        return false;
    }
    char *rxData = serialBuffer; //         and then convert it to data types acceptable by the
    int rxSize = ret;            //         Communication Abstraction Layer (CAL).
    Ep_Header header;
    while (EP_SUCC_ == eP.On_RecvPkg(rxData, rxSize, &header))
    {               // Step 2: Tell the CAL that new data has arrived.
                    //         It does not matter if the new data only contains a fraction
                    //         of a complete package, nor does it matter if the data is broken
                    //         during the transmission. On_RecvPkg() will only return EP_SUCC_
                    //         when a complete and correct package has arrived.
        rxData = 0; //         The while loop ensures no package is omitted.
        rxSize = 0;
        // Example Reading of the Short ID of the device who send the data:
        uint32 fromId = header.fromId; // Step 3.1:  Now we are able to read the received payload data.
                                       //            header.fromId tells us from which Motion Module the data comes.

        // Supress "parameter unused" complier warning:
        (void)fromId;

        switch (header.cmd)
        { // Step 3.2: header.cmd tells what kind of data is inside the payload.
        case EP_CMD_ACK_:
        {                  //           We can use a switch() as demonstrated here to do different
            Ep_Ack ep_Ack; //           tasks for different types of data.
            if (EP_SUCC_ == eOD.Read_Ep_Ack(&ep_Ack))
            {
            }
        }
        break;
        case EP_CMD_STATUS_:
        {
            Ep_Status ep_Status;
            if (EP_SUCC_ == eOD.Read_Ep_Status(&ep_Status))
            {
            }
        }
        break;
        case EP_CMD_Raw_GYRO_ACC_MAG_:
        {
            Ep_Raw_GyroAccMag ep_Raw_GyroAccMag;
            if (EP_SUCC_ == eOD.Read_Ep_Raw_GyroAccMag(&ep_Raw_GyroAccMag))
            {
                float gyro_x = ep_Raw_GyroAccMag.gyro[0];
                float gyro_y = ep_Raw_GyroAccMag.gyro[1];
                float gyro_z = ep_Raw_GyroAccMag.gyro[2];

                float acc_x = ep_Raw_GyroAccMag.acc[0] * 9.8f;
                float acc_y = ep_Raw_GyroAccMag.acc[1] * 9.8f;
                float acc_z = ep_Raw_GyroAccMag.acc[2] * 9.8f;

                float mag_x = ep_Raw_GyroAccMag.mag[0];
                float mag_y = ep_Raw_GyroAccMag.mag[1];
                float mag_z = ep_Raw_GyroAccMag.mag[2];

                imu_data_msg.angular_velocity.x = gyro_x;
                imu_data_msg.angular_velocity.y = gyro_y;
                imu_data_msg.angular_velocity.z = gyro_z;

                imu_data_msg.linear_acceleration.x = acc_x;
                imu_data_msg.linear_acceleration.y = acc_y;
                imu_data_msg.linear_acceleration.z = acc_z;
#ifdef ROS_MAG_MSG
                imu_data_mag.magnetic_field.x = mag_x;
                imu_data_mag.magnetic_field.y = mag_y;
                imu_data_mag.magnetic_field.z = mag_z;

#endif
#ifdef DEBUG_MODE
                ROS_INFO("RAW pkg");
#endif
            }
        }
        break;
        case EP_CMD_Q_S1_S_:
        {
            Ep_Q_s1_s ep_Q_s1_s;
            if (EP_SUCC_ == eOD.Read_Ep_Q_s1_s(&ep_Q_s1_s))
            {
            }
        }
        break;
        case EP_CMD_Q_S1_E_:
        {
            Ep_Q_s1_e ep_Q_s1_e;
            if (EP_SUCC_ == eOD.Read_Ep_Q_s1_e(&ep_Q_s1_e))
            { // Step 3.3: If we decided that the received Quaternion should be used,
              //           Here is an example of how to access the Quaternion data.
                float q1 = ep_Q_s1_e.q[0];
                float q2 = ep_Q_s1_e.q[1];
                float q3 = ep_Q_s1_e.q[2];
                float q4 = ep_Q_s1_e.q[3];
                // uint32 timeStamp = ep_Q_s1_e.timeStamp;    //           TimeStamp indicates the time point (since the Module has been powered on),
                //            when this particular set of Quaternion was calculated. (Unit: uS)
                //            Note that overflow will occure when the uint32 type reaches its maximum value.
                // uint32 deviceId = ep_Q_s1_e.header.fromId; //           The ID indicates the device Short ID telling which Motion Module the data comes from.

                imu_data_msg.orientation.w = q1;
                imu_data_msg.orientation.x = q2;
                imu_data_msg.orientation.y = q3;
                imu_data_msg.orientation.z = q4;
#ifdef DEBUG_MODE
                ROS_INFO("QS1 pkg");
#endif
            }
        }
        break;
        case EP_CMD_EULER_S1_S_:
        {
            Ep_Euler_s1_s ep_Euler_s1_s;
            if (EP_SUCC_ == eOD.Read_Ep_Euler_s1_s(&ep_Euler_s1_s))
            {
            }
        }
        break;
        case EP_CMD_EULER_S1_E_:
        {
            Ep_Euler_s1_e ep_Euler_s1_e;
            if (EP_SUCC_ == eOD.Read_Ep_Euler_s1_e(&ep_Euler_s1_e))
            {
            }
        }
        break;
        case EP_CMD_RPY_:
        {
            Ep_RPY ep_RPY;
            if (EP_SUCC_ == eOD.Read_Ep_RPY(&ep_RPY))
            { //           Another Example reading of the received Roll Pitch and Yaw
                float roll = ep_RPY.roll;
                float pitch = ep_RPY.pitch;
                float yaw = ep_RPY.yaw;
                // uint32 timeStamp = ep_RPY.timeStamp;    //           TimeStamp indicates the time point (since the Module has been powered on),
                //            when this particular set of Roll-Pitch-Yaw was calculated. (Unit: uS)
                //            Note that overflow will occure when the uint32 type reaches its maximum value.
                // uint32 deviceId = ep_RPY.header.fromId; //           The ID indicates from wich IMU Module the data comes from.
#ifdef ROS_RPY_MSG
                imu_data_rpy.magnetic_field.x = roll;
                imu_data_rpy.magnetic_field.y = pitch;
                imu_data_rpy.magnetic_field.z = yaw;
#endif
#ifdef DEBUG_MODE
                ROS_INFO("RPY pkg");
#endif
            }
        }
        break;
        case EP_CMD_GRAVITY_:
        {
            Ep_Gravity ep_Gravity;
            if (EP_SUCC_ == eOD.Read_Ep_Gravity(&ep_Gravity))
            {
            }
        }
        break;
        case EP_CMD_COMBO_:
        {
            Ep_Combo ep_Combo;
            if (EP_SUCC_ == eOD.Read_Ep_Combo(&ep_Combo))
            {
                // Accelerometer:
                float ax = (ep_Combo.ax) * (1e-5f); // Unit: 1g, 1g = 9.794m/(s^2)
                float ay = (ep_Combo.ay) * (1e-5f);
                float az = (ep_Combo.az) * (1e-5f);
                // Gyroscope:
                float wx = (ep_Combo.wx) * (1e-5f); // Unit: rad/s
                float wy = (ep_Combo.wy) * (1e-5f);
                float wz = (ep_Combo.wz) * (1e-5f);
                // Magnetometer:
                float mx = (ep_Combo.mx) * (1e-3f); // Unit: one earth magnetic field
                float my = (ep_Combo.my) * (1e-3f); // vector (mx, my, mz) is used as direction reference of the local magnetic field.
                float mz = (ep_Combo.mz) * (1e-3f);
                // Quaternion in (q1,q2,q3,q4)=(w,x,y,z) format
                float q1 = (ep_Combo.q1) * (1e-7f);
                float q2 = (ep_Combo.q2) * (1e-7f);
                float q3 = (ep_Combo.q3) * (1e-7f);
                float q4 = (ep_Combo.q4) * (1e-7f);
                // RPY:
                float roll = (ep_Combo.roll) * (1e-2f);   // Unit: degree
                float pitch = (ep_Combo.pitch) * (1e-2f); // Unit: degree
                float yaw = (ep_Combo.yaw) * (1e-2f);     // Unit: degree

                imu_data_msg.angular_velocity.x = wx;
                imu_data_msg.angular_velocity.y = wy;
                imu_data_msg.angular_velocity.z = wz;

                imu_data_msg.linear_acceleration.x = ax;
                imu_data_msg.linear_acceleration.y = ay;
                imu_data_msg.linear_acceleration.z = az;

                imu_data_msg.orientation.x = q1;
                imu_data_msg.orientation.y = q2;
                imu_data_msg.orientation.z = q3;
                imu_data_msg.orientation.w = q4;
#ifdef ROS_RPY_MSG
                imu_data_rpy.magnetic_field.x = roll;
                imu_data_rpy.magnetic_field.y = pitch;
                imu_data_rpy.magnetic_field.z = yaw;
#endif
#ifdef ROS_MAG_MSG
                imu_data_mag.magnetic_field.x = mx;
                imu_data_mag.magnetic_field.y = my;
                imu_data_mag.magnetic_field.z = mz;
#endif

#ifdef DEBUG_MODE
                ROS_INFO("Combo pkg");
#endif
            }
        }
        break;
        }
    }
    return true;
}
// Serial Receive Data Example
//----------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    TMSerial tmSerial1;
    // Open serial port
    tmSerial1.Serialport_Open();

    ros::init(argc, argv, "tmSerial");
    ros::NodeHandle nh;
    ros::Publisher TransducerM_pub_imu = nh.advertise<sensor_msgs::Imu>("TransducerM_imu_msg", 100);
#ifdef ROS_MAG_MSG
    ros::Publisher TransducerM_pub_mag = nh.advertise<sensor_msgs::MagneticField>("TransducerM_imu_mag", 100);
#endif
#ifdef ROS_RPY_MSG
    ros::Publisher TransducerM_pub_rpy = nh.advertise<sensor_msgs::MagneticField>("TransducerM_imu_rpy", 100);
#endif
    tmSerial1.imu_data_msg.header.frame_id = "imu_data";
#ifdef ROS_MAG_MSG
    tmSerial1.imu_data_mag.header.frame_id = "imu_mag";
#endif
#ifdef ROS_RPY_MSG
    tmSerial1.imu_data_rpy.header.frame_id = "imu_rpy";
#endif
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        tmSerial1.On_SerialRX();
        tmSerial1.imu_data_msg.header.stamp = ros::Time::now();
        TransducerM_pub_imu.publish(tmSerial1.imu_data_msg);
#ifdef ROS_MAG_MSG
        tmSerial1.imu_data_mag.header.stamp = ros::Time::now();
        TransducerM_pub_mag.publish(tmSerial1.imu_data_mag);
#endif
#ifdef ROS_RPY_MSG
        tmSerial1.imu_data_rpy.header.stamp = ros::Time::now();
        TransducerM_pub_rpy.publish(tmSerial1.imu_data_rpy);
#endif
#ifdef DEBUG_MODE
        ROS_INFO("Run once");
#endif
        // ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
