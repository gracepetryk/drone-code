#include <radio_config_Si4460.h>
#include <RadioHead.h>
#include <RH_ASK.h>
#include <RH_NRF24.h>
#include <RH_NRF905.h>
#include <RH_RF22.h>
#include <RH_RF24.h>
#include <RH_RF69.h>
#include <RH_RF95.h>
#include <RH_Serial.h>
#include <RH_TCP.h>
#include <RHCRC.h>
#include <RHDatagram.h>
#include <RHGenericDriver.h>
#include <RHGenericSPI.h>
#include <RHHardwareSPI.h>
#include <RHMesh.h>
#include <RHNRFSPIDriver.h>
#include <RHReliableDatagram.h>
#include <RHRouter.h>
#include <RHSoftwareSPI.h>
#include <RHSPIDriver.h>
#include <RHTcpProtocol.h>

#include <SoftwareServo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <MadgwickAHRS.h>
#include <PID_AutoTune_v0.h>
#include <PID_v1.h>
#include <Wire.h>
#include <SPI.h>


/* Authors: John Petryk and Henryk Viana
 * Arduino code for running the drone
 */

RH_ASK driver; // coms driver

//pins
SoftwareServo frontMotor;
SoftwareServo backMotor;
SoftwareServo rightMotor;
SoftwareServo leftMotor;

Madgwick filter;

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

//pid perameters
double pitchSetpoint, pitchInput, pitchMultiplier; // pitch PID perameters 
double yawSetpoint, yawInput, yawMultiplier; // yaw PID perameters
double rollSetpoint, rollInput, rollMultiplier; // roll PID perameters
double altitudeSetpoint, altitudeInput, altitudeMultiplier; // altitude PID perameters

//PID objects - & symbols for global varables
PID pitchPID(&pitchInput, &pitchMultiplier, &pitchSetpoint, 1, 0, 0, DIRECT);
PID yawPID(&yawInput, &yawMultiplier, &yawSetpoint, 1, 0, 0, DIRECT);
PID rollPID(&rollInput, &rollMultiplier, &rollSetpoint, 1, 0, 0, DIRECT);

float PIDmin = 0.8;
float PIDmax = 1.2;

float powerMultiplier = 1; // min 0, max 1

void setup() 
{
    //initialize Comms

    if(!driver.init())
    {
        Serial.println("init failed");
    }
    
    // initialize pins (order is clockwise)
    frontMotor.attach(4);
    rightMotor.attach(5);
    backMotor.attach(6);
    leftMotor.attach(7);

    // intialize PID's
    pitchPID.SetMode(AUTOMATIC);
    yawPID.SetMode(AUTOMATIC);
    rollPID.SetMode(AUTOMATIC);

    //set PID limits
    pitchPID.SetOutputLimits(PIDmin, PIDmax);
    yawPID.SetOutputLimits(PIDmin, PIDmax);
    rollPID.SetOutputLimits(PIDmin, PIDmax);

    //initialize gyro
    lsm.begin();
    filter.begin(25);
}


void applyMotorPower()
{
    double frontPitch = pitchMultiplier;
    double frontYaw = yawMultiplier;

    double leftRoll = rollMultiplier;
    double leftYaw = yawMultiplier;

    double backPitch = pitchMultiplier;
    double backYaw = yawMultiplier;

    double rightRoll = pitchMultiplier;
    double rightYaw = yawMultiplier;

    if (pitchMultiplier >= 1)
    {
        frontPitch = 1 - (pitchMultiplier - 1); 
    } else {
        backPitch = 1 - (pitchMultiplier - 1);
    }

    if (yawMultiplier >= 1)
    {
        leftYaw, rightYaw = 1 - (yawMultiplier - 1);
    } else {
        frontYaw, leftYaw = 1 - (yawMultiplier - 1);
    }

    if (rollMultiplier >= 1)
    {
        leftRoll = 1 - (rollMultiplier - 1);
    } else {
        rightRoll = 1 - (rollMultiplier - 1);
    }

    frontMotor.write(map(frontPitch*frontYaw*powerMultiplier, 0, 1.44, 30, 130));
    rightMotor.write(map(rightRoll*rightYaw*powerMultiplier, 0, 1.44, 30, 130));
    backMotor.write(map(backPitch*backYaw*powerMultiplier, 0, 1.44, 30, 130));
    leftMotor.write(map(leftRoll*leftYaw*powerMultiplier, 0, 1.44, 30, 130));


}



void readComms(String &msg)
{
    uint8_t buf[64];
    uint8_t buflen = sizeof(buf);
    if (driver.recv(buf, &buflen))
    {
        msg = String((char*)buf);
    }
}

void loop()
{

    
}
