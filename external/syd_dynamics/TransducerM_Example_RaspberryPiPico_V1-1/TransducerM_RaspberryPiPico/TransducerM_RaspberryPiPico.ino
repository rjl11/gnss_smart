#include <EasyObjectDictionary.h>
#include <EasyProfile.h>

EasyObjectDictionary eOD;
EasyProfile eP(&eOD);
#define AUTOFIX_TIME (500)     // ms
#define CALIBFORCE_TIME (1000) // ms
#define SERIAL_BUFFER_SIZE (64)
char rxData_Serial[SERIAL_BUFFER_SIZE] = {0};
char CalibSuccessFlag = 0;
void TX_CalibCompass_AutoFix_Start();
void TX_CalibCompass_AutoFix_Stop();
void TX_CalibForce_Continuous();
void TX_CalibForce_Finish();

void setup()
{
    Serial.begin(115200);  // Use Serial for Debug print
    Serial1.begin(115200); // Use Serial1 to interface with TransducerM (Useing Raspberry Pi Pico GP0 as UART0_TX GP1 as UART0 RX)
}

void loop()
{
    // Example call of digital compass API (Find north once upon TransducerM power on)
    if (!CalibSuccessFlag)
    {
        TX_CalibCompass_AutoFix_Start();
        delay(AUTOFIX_TIME);
        TX_CalibCompass_AutoFix_Stop();
    }
    
      // Example call of Forced Calibration (Run-Time Static Calibration) API to manually remove gyroscope bias
    if (0)                       // Please implement your own logic to trigger this function. Example triggering logic: 
    {                            //       when the device being measured has stopped moving completely. You may also refer to application note (DDTM-AN0002).
        TX_CalibForce_Continuous();
        delay(CALIBFORCE_TIME);  // TransducerM will do data collection during this period. 
                                 //      Caution!  TransducerM must be stationary during this period (i.e. stop moving before TX_CalibForce_Continuous() call and until TransducerM returns success message after TX_CalibForce_Finish() call. 
        TX_CalibForce_Finish();  // Gyro bias will be removed according to the stationary data collected.
    }

    // TransducerM Data parsing
    On_SerialRX();
}

void On_SerialRX(void)
{
    int rxSize = Serial1.available();
    if (rxSize > 0)
    {
        if (rxSize > SERIAL_BUFFER_SIZE)
            rxSize = SERIAL_BUFFER_SIZE;
        rxSize = Serial1.readBytes(rxData_Serial, rxSize);
        Serial_ISR(rxData_Serial, rxSize);
        Serial_ISR(0, 0);
        Serial_ISR(0, 0);
        Serial_ISR(0, 0);
    }
}

void Serial_ISR(char *rxData, int rxSize)
{
    Ep_Header header;
    if (EP_SUCC_ == eP.On_RecvPkg(rxData, rxSize, &header))
    {
        uint32 fromId = header.fromId;
        (void)fromId;

        switch (header.cmd)
        {
        case EP_CMD_ACK_:
        {
            Ep_Ack ep_Ack;
            if (EP_SUCC_ == eOD.Read_Ep_Ack(&ep_Ack))
            {
            }
        }
        break;
        case EP_CMD_RPY_:
        {
            Ep_RPY ep_RPY;
            if (EP_SUCC_ == eOD.Read_Ep_RPY(&ep_RPY))
            { //            Example reading of the received Roll Pitch and Yaw
                float roll = ep_RPY.roll;
                float pitch = ep_RPY.pitch;
                float yaw = ep_RPY.yaw;
                uint32 timeStamp = ep_RPY.timeStamp;    //           TimeStamp indicates the time point (since the TransducerM has been powered on),
                                                        //           when this particular set of Roll-Pitch-Yaw was calculated. (Unit: uS)
                                                        //           Note that overflow will occure when the uint32 type reaches its maximum value.
                uint32 deviceId = ep_RPY.header.fromId; //           The ID indicates from wich TransducerM the data comes from.
                Serial.print("roll = ");
                Serial.print(roll);
                Serial.print("pitch = ");
                Serial.print(pitch);
                Serial.print("yaw = ");
                Serial.print(yaw);
                Serial.print("\n");
            }
        }
        break;
        case EP_CMD_Raw_GYRO_ACC_MAG_:
        {
            Ep_Raw_GyroAccMag ep_Raw_GyroAccMag;
            if (EP_SUCC_ == eOD.Read_Ep_Raw_GyroAccMag(&ep_Raw_GyroAccMag))
            {
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
                uint32 timeStamp = ep_Q_s1_e.timeStamp;    //           TimeStamp indicates the time point (since the TransducerM has been powered on),
                                                           //           when this particular set of Quaternion was calculated. (Unit: uS)
                                                           //           Note that overflow will occure when the uint32 type reaches its maximum value.
                uint32 deviceId = ep_Q_s1_e.header.fromId; //           The ID indicates the device Short ID telling which TransducerM the data comes from.
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
        case EP_CMD_GRAVITY_:
        {
            Ep_Gravity ep_Gravity;
            if (EP_SUCC_ == eOD.Read_Ep_Gravity(&ep_Gravity))
            {
            }
        }
        case EP_CMD_CALIB_:
        {
            Ep_Calib ep_Calib;
            if (EP_SUCC_ == eOD.Read_Ep_Calib(&ep_Calib))
            {
                uint32 deviceId = ep_Calib.header.fromId;
                uint8 calibType = ep_Calib.calibType;
                uint16 status = ep_Calib.ctrlVal;

                (void)deviceId; // Remove 'variable unused' warning from compiler

                //--------------------------------------------
                // CalibCompass status report
                if (calibType == (3 + 0x10))
                { // 3 means calibration type is CompassCalib.  +0x10 means it is its status report.
                    if (status == 1)
                    {
                        Serial.print("Compass Calib: Operation Successfull");
                        Serial.print("\n");
                        CalibSuccessFlag = 1;
                    }
                    else
                    {
                        Serial.print("Compass Calib: Operation Failed.");
                        Serial.print("\n");
                    }
                }
                // CalibCompass status report
                //--------------------------------------------

                //--------------------------------------------
                // CalibB status report
                else if (calibType == (4 + 0x10))
                { // 4 means calibration type is CalibB.  +0x10 means it is its status report.
                    if (status == 1)
                    { // calibration successful.
                        Serial.print("CalibB Successful!");
                        Serial.print("\n");
                    }
                    else
                    {
                        Serial.print("CalibB Failed!");
                        Serial.print("\n");
                    }
                }
                // CalibB status report
                //--------------------------------------------

                //--------------------------------------------
                // Forced Calibration status report
                else if (calibType == (5 + 0x10))
                { // 5 means calibration type is Force Calibration,  +0x10 means it is its status report.
                    if (status == 0)
                    { // calibration successful.
                        Serial.print("Forced Calibration Processing..");
                        Serial.print("\n");
                    }
                    else if (status == 1)
                    {
                        Serial.print("Forced Calibration Successful!");
                        Serial.print("\n");
                    }
                    else if (status == 5)
                    {
                        Serial.print("Forced Calibration successfully revoked!");
                        Serial.print("\n");
                    }
                    else
                    {
                        Serial.print("Forced Calibration Failed!");
                        Serial.print("\n");
                    }
                }
                // Forced Calibration status report
                //--------------------------------------------
            }
        }
        break;
        } // switch (header.cmd)
    }
}

/**
 * Start collecting data for correcting heading (Yaw) using local magnetic field measured by magnetometer.
 * @version [This Example applys to TransducerM TM5xx series models only]
 * @note    It is recommended, if possible, during the data collection process to rotate the TransducerM
 *          around Z axis (axis perpendicular to the horizontal plane)  to increase the accuracy of results;
 *          the rotation can be a small angle such as 3-5 degree, 30 degrees, or a large angle such as a full turn or more).
 *          The results are saved in TransducerM RAM and will be lost during power cycle.
 */
void TX_CalibCompass_AutoFix_Start()
{
    // uint16 toId = ui->spinBox_TX_Request_ShortId->value();// Step 1: Get the destination device ID to which the request is sent
    uint16 toId = EP_ID_BROADCAST_; //         You can also choose to boardcast the request without specifying the target device ID

    // Step 2 : Set parameter
    uint32 calibrationTime = 90000; //         Unit: ms. Maximum calibration time. Range (0, 90000] reflecting 0~90 seconds. 0 means infinite time.
                                    //
    if (EP_SUCC_ == eOD.Write_Ep_Calib(toId, 3, 0x0a, 0xaa32, calibrationTime, 0))
    { //         The parameter value "3, 0x0a, 0xaa32" and "0" are fixed and do not change them.
        EP_ID_TYPE_ txToId;
        char *txPkgData;
        int txPkgSize;
        EP_CMD_TYPE_ txCmd = EP_CMD_CALIB_; // Step 3: Define what type of data we are sending (Here we must declare that it is calibration command)
        if (EP_SUCC_ == eP.On_SendPkg(txCmd, &txToId, &txPkgData, &txPkgSize))
        {                                        // Step 4: Create a package out of the data structure (i.e. the payload) to be sent
            Serial1.write(txPkgData, txPkgSize); // Step 5: Send the package via Serial Port
        }
    }
}

/**
 * Finish collecting data for correcting heading (Yaw) using magnetitic north measured by magnetometer and apply results.
 * @version [This Example applys to TransducerM TM5xx series models only]
 * @note    When the AutoFix is finished, a status report is sent back to the host, telling whether it is
 *          successful or not. Please refer to the EP_CMD_CALIB_ branch of function on_SerialRX() defined below.
 *          The longer time interval between AutoFix Start and Stop command, the better the results will be. It
 *          is recommended to last at least 2 seconds before Stop command is sent.
 */
void TX_CalibCompass_AutoFix_Stop()
{
    // uint16 toId = ui->spinBox_TX_Request_ShortId->value(); // Step 1: Get the destination device ID to which the request is sent
    uint16 toId = EP_ID_BROADCAST_; //         You can also choose to boardcast the request without specifying the target device ID

    // Step 2 : Set parameter
    if (EP_SUCC_ == eOD.Write_Ep_Calib(toId, 3, 0x0b, 0xaa32, 0, 0))
    { //         The parameter value "3, 0x0b, 0xaa32, 0, 0" are fixed and do not change them.
        EP_ID_TYPE_ txToId;
        char *txPkgData;
        int txPkgSize;
        EP_CMD_TYPE_ txCmd = EP_CMD_CALIB_; // Step 3: Define what type of data we are sending (Here we must declare that it is calibration command)
        if (EP_SUCC_ == eP.On_SendPkg(txCmd, &txToId, &txPkgData, &txPkgSize))
        {                                        // Step 4: Create a package out of the data structure (i.e. the payload) to be sent
            Serial1.write(txPkgData, txPkgSize); // Step 5: Send the package via Serial Port
        }
    }
}

/**
 * @brief    Start continuous Forced Calibration (a.k.a Run-time Static Calibration)
 * @version  This function is available with TransducerM with firmware version V5.1.x and later (except V2x.x.x).
 * @see      Refer to user guide (Chapter 'Write Your Own Communication Library/EasyProtocol/ObjectTypes/Calibration') for details.
 *
 * @note Calibration status reports are sent back to the host, telling whether it is successful or not.
 *       Please refer to the EP_CMD_CALIB_ branch of function on_SerialRX() defined below.
 *       If the host does not get a status report (very rare, but it is possible that either the calibration
 *       command or the status report package is lost during the communication), Simply send the command again.
 */
void TX_CalibForce_Continuous()
{
    // uint16 toId = ui->spinBox_TX_Request_ShortId->value(); // Step 1: Get the destination device ID to which the request is sent
    uint16 toId = EP_ID_BROADCAST_; //         You can also choose to boardcast the request without specifying the target device ID

    float calibLimit = 1.5;    //         Set calibLimit between 0.05 and 5.00 degree/s. Motion data within the range of +-calibLimit will be accepted as valid calibration data.
    float calibConfidence = 0; //         Set calibConfidence between 0 and 100 (meaning 0% - 100%). 0 means auto confidence mode.
    if (EP_SUCC_ == eOD.Write_Ep_Calib(toId, 5, 3, 0xaa65, (uint32)(calibLimit * 100), (uint32)(calibConfidence * 100)))
    {
        //         the parameter value "5, 3, 0xaa65" are fixed and do not change them.
        //         the parameter "(uint32)(calibLimit*100)" is a conversion and do not change it.
        //         the parameter "(uint32)(calibConfidence*100)" is a conversion and do not change it.
        EP_ID_TYPE_ txToId;
        char *txPkgData;
        int txPkgSize;
        EP_CMD_TYPE_ txCmd = EP_CMD_CALIB_; // Step 3: Define what type of data we are sending (Here we must declare that it is calibration command)
        if (EP_SUCC_ == eP.On_SendPkg(txCmd, &txToId, &txPkgData, &txPkgSize))
        {                                        // Step 4: Create a package out of the data structure (i.e. the payload) to be sent
            Serial1.write(txPkgData, txPkgSize); // Step 5: Send the package via Serial Port
        }
    }
}

/**
 * @brief    Finish continuous Forced Calibration (a.k.a Run-time Static Calibration).
 * @version  This function is available with TransducerM with firmware version V5.1.x and later (except V2x.x.x).
 * @see      Refer to user guide (Chapter 'Write Your Own Communication Library/EasyProtocol/ObjectTypes/Calibration') for details.
 *
 * @note Calibration status reports are sent back to the host, telling whether it is successful or not.
 *       Please refer to the EP_CMD_CALIB_ branch of function on_SerialRX() defined below.
 */
void TX_CalibForce_Finish()
{
    // uint16 toId = ui->spinBox_TX_Request_ShortId->value(); // Step 1: Get the destination device ID to which the request is sent
    uint16 toId = EP_ID_BROADCAST_; //         You can also choose to boardcast the request without specifying the target device ID

    if (EP_SUCC_ == eOD.Write_Ep_Calib(toId, 5, 4, 0xaa65, 0, 0))
    {
        //         the parameter value "5, 4, 0xaa65" are fixed and do not change them.
        EP_ID_TYPE_ txToId;
        char *txPkgData;
        int txPkgSize;
        EP_CMD_TYPE_ txCmd = EP_CMD_CALIB_; // Step 3: Define what type of data we are sending (Here we must declare that it is calibration command)
        if (EP_SUCC_ == eP.On_SendPkg(txCmd, &txToId, &txPkgData, &txPkgSize))
        {                                        // Step 4: Create a package out of the data structure (i.e. the payload) to be sent
            Serial1.write(txPkgData, txPkgSize); // Step 5: Send the package via Serial Port
        }
    }
}
