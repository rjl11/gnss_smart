#include <SoftwareSerial.h>

// Motion Module Interface:
#include <EasyObjectDictionary.h>
#include <EasyProfile.h>
EasyObjectDictionary eOD;
EasyProfile          eP(&eOD);

SoftwareSerial softSerial(10, 11); 

void setup() {
  Serial.begin(115200);      // For print()
  softSerial.begin(115200);  // For communication with the Motion Module
}

void loop() {
  SerialRX(); //call the function
}


void SerialRX() {
  while (softSerial.available()) {
    // Read the received byte:
    char rxByte = (char)softSerial.read();                     // Step 1: read the received data buffer of the Serial Port
    char*  rxData = &rxByte;                                   //         and then convert it to data types acceptable by the
    int    rxSize = 1;                                         //         Communication Abstraction Layer (CAL).
    Ep_Header header;
    if(EP_SUCC_ == eP.On_RecvPkg(rxData, rxSize, &header)){    // Step 2: Tell the CAL that new data has arrived.
                                                               //         It does not matter if the new data only contains a fraction
                                                               //         of a complete package, nor does it matter if the data is broken
                                                               //         during the transmission. On_RecvPkg() will only return EP_SUCC_
                                                               //         when a complete and correct package has arrived.

        // Example Reading of the Short ID of the device who send the data:
        uint32 fromId = header.fromId;                         // Step 3.1:  Now we are able to read the received payload data.
                                                               //            header.fromId tells us from which Motion Module the data comes.

        //Supress "parameter unused" complier warning:
        (void)fromId;

        switch (header.cmd) {                                  // Step 3.2: header.cmd tells what kind of data is inside the payload.
        case EP_CMD_ACK_:{                                     //           We can use a switch() as demonstrated here to do different
            Ep_Ack ep_Ack;                                     //           tasks for different types of data.
            if(EP_SUCC_ == eOD.Read_Ep_Ack(&ep_Ack)){

            }
        }break;
        case EP_CMD_STATUS_:{
            Ep_Status ep_Status;
            if(EP_SUCC_ == eOD.Read_Ep_Status(&ep_Status)){

            }
        }break;
        case EP_CMD_Raw_GYRO_ACC_MAG_:{
            Ep_Raw_GyroAccMag ep_Raw_GyroAccMag;
            if(EP_SUCC_ == eOD.Read_Ep_Raw_GyroAccMag(&ep_Raw_GyroAccMag)){

            }
        }break;
        case EP_CMD_Q_S1_S_:{
            Ep_Q_s1_s ep_Q_s1_s;
            if(EP_SUCC_ == eOD.Read_Ep_Q_s1_s(&ep_Q_s1_s)){

            }
        }break;
        case EP_CMD_Q_S1_E_:{
            Ep_Q_s1_e ep_Q_s1_e;
            if(EP_SUCC_ == eOD.Read_Ep_Q_s1_e(&ep_Q_s1_e)){ // Step 3.3: If we decided that the received Quaternion should be used,
                                                            //           Here is an example of how to access the Quaternion data.
                float q1 = ep_Q_s1_e.q[0];
                float q2 = ep_Q_s1_e.q[1];
                float q3 = ep_Q_s1_e.q[2];
                float q4 = ep_Q_s1_e.q[3];
                uint32 timeStamp = ep_Q_s1_e.timeStamp;     //           TimeStamp indicates the time point (since the Module has been powered on),
                                                            //           when this particular set of Quaternion was calculated. (Unit: uS)
                                                            //           Note that overflow will occure when the uint32 type reaches its maximum value.
                uint32 deviceId  = ep_Q_s1_e.header.fromId; //           The ID indicates the device Short ID telling which Motion Module the data comes from.

                // Display the data:
                Serial.print("Quaternion: "); 
                Serial.print(q1); Serial.print(", ");
                Serial.print(q2); Serial.print(", ");
                Serial.print(q3); Serial.print(", ");
                Serial.print(q4); 
                Serial.print("  TimeStamp:");      Serial.print(timeStamp);
                Serial.print("  Device Short ID:");Serial.println(deviceId);
            }
        }break;
        case EP_CMD_EULER_S1_S_:{
            Ep_Euler_s1_s ep_Euler_s1_s;
            if(EP_SUCC_ == eOD.Read_Ep_Euler_s1_s(&ep_Euler_s1_s)){

            }
        }break;
        case EP_CMD_EULER_S1_E_:{
            Ep_Euler_s1_e ep_Euler_s1_e;
            if(EP_SUCC_ == eOD.Read_Ep_Euler_s1_e(&ep_Euler_s1_e)){

            }
        }break;
        case EP_CMD_RPY_:{
            Ep_RPY ep_RPY;
            if(EP_SUCC_ == eOD.Read_Ep_RPY(&ep_RPY)){     //           Another Example reading of the received Roll Pitch and Yaw
                float roll  = ep_RPY.roll;
                float pitch = ep_RPY.pitch;
                float yaw   = ep_RPY.yaw;
                uint32 timeStamp = ep_RPY.timeStamp;      //           TimeStamp indicates the time point (since the Module has been powered on),
                                                          //           when this particular set of Roll-Pitch-Yaw was calculated. (Unit: uS)
                                                          //           Note that overflow will occure when the uint32 type reaches its maximum value.
                uint32 deviceId  = ep_RPY.header.fromId;  //           The ID indicates from wich IMU Module the data comes from.
                // Display the data:
                Serial.print("Roll:");  Serial.print(roll); 
                Serial.print(" Pitch:");Serial.print(pitch); 
                Serial.print(" Yaw");   Serial.print(yaw); 
                Serial.print("  TimeStamp:");      Serial.print(timeStamp);
                Serial.print("  Device Short ID:");Serial.println(deviceId);
            }
        }break;
        case EP_CMD_GRAVITY_:{
            Ep_Gravity ep_Gravity;
            if(EP_SUCC_ == eOD.Read_Ep_Gravity(&ep_Gravity)){

            }
        }break;
        case EP_CMD_COMBO_:{
            Ep_Combo ep_Combo;
            if(EP_SUCC_ == eOD.Read_Ep_Combo(&ep_Combo)){
                // Example reading of the received data:
                // Accelerometer:
                float ax = (ep_Combo.ax)*(1e-5f);                     // Unit: 1g, 1g = 9.794m/(s^2)
                float ay = (ep_Combo.ay)*(1e-5f);
                float az = (ep_Combo.az)*(1e-5f);
                // Gyroscope:
                float wx = (ep_Combo.wx)*(1e-5f);                     // Unit: rad/s
                float wy = (ep_Combo.wy)*(1e-5f);
                float wz = (ep_Combo.wz)*(1e-5f);
                // Magnetometer:
                float mx = (ep_Combo.mx)*(1e-3f);                     // Unit: one earth magnetic field
                float my = (ep_Combo.my)*(1e-3f);                     // vector (mx, my, mz) is used as direction reference of the local magnetic field.
                float mz = (ep_Combo.mz)*(1e-3f);
                // Quaternion in (q1,q2,q3,q4)=(w,x,y,z) format
                float q1 = (ep_Combo.q1)*(1e-7f);
                float q2 = (ep_Combo.q2)*(1e-7f);
                float q3 = (ep_Combo.q3)*(1e-7f);
                float q4 = (ep_Combo.q4)*(1e-7f);
                // RPY:
                float roll  = (ep_Combo.roll)*(1e-2f);                // Unit: degree
                float pitch = (ep_Combo.pitch)*(1e-2f);               // Unit: degree
                float yaw   = (ep_Combo.yaw)*(1e-2f);                 // Unit: degree

                Serial.print("Accelerometer: "); Serial.print(ax); Serial.print(", "); Serial.print(ay); Serial.print(", "); Serial.println(az);
                Serial.print("Gyroscope: ");     Serial.print(wx); Serial.print(", "); Serial.print(wy); Serial.print(", "); Serial.println(wz);
                Serial.print("Magnetometer: ");  Serial.print(mx); Serial.print(", "); Serial.print(my); Serial.print(", "); Serial.println(mz);
                Serial.print("Quaternion: ");    Serial.print(q1); Serial.print(", "); Serial.print(q2); Serial.print(", "); Serial.print(q3); Serial.print(", "); Serial.println(q4);
                Serial.print("RPY: ");           Serial.print(roll); Serial.print(", "); Serial.print(pitch); Serial.print(", "); Serial.println(yaw);

            }
        }

    }

  }
}
