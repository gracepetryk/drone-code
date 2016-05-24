#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <MadgwickAHRS.h>
#include <PID_AutoTune_v0.h>
#include <Servo.h>
#include <PID_v1.h>
#include <Wire.h>
#include <SPI.h>

/* Authors: John Petryk and Henryk Viana
 * Arduino code for running the drone
 */

//pins
Servo frontMotor;
Servo backMotor;
Servo rightMotor;
Servo leftMotor;

Madgwick filter;

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

//pid perameters
double pitchSetpoint, pitchInput, pitchOutput; // pitch PID perameters 
double yawSetpoint, yawInput, yawOutput; // yaw PID perameters
double rollSetpoint, rollInput, rollOutput; // roll PID perameters
double altitudeSetpoint, altitudeInput, altitudeOutput; // altitude PID perameters

//PID objects - & symbols for global varables
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, 1, 0, 0, DIRECT);
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, 1, 0, 0, DIRECT);
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, 1, 0, 0, DIRECT);

float PIDmin = 1;
float PIDmax = sqrt(1.5);

float powerMultiplier = 0; // min 0, max 1

void setup() 
{
    // initialize pins
    frontMotor.attach(13);
    backMotor.attach(12);
    rightMotor.attach(11);
    backMotor.attach(10);

    // intialize PID's
    pitchPID.SetMode(AUTOMATIC);
    yawPID.SetMode(AUTOMATIC);
    rollPID.SetMode(AUTOMATIC);

    //set PID limits
    pitchPID.SetOutputLimits(PIDmin, PIDmax);
    yawPID.SetOutputLimits(PIDmin, PIDmax);
    rollPID.SetOutputLimits(PIDmin, PIDmax);

    lsm.begin();

    filter.begin(25);
}


void applyMotorPower(PID pid, Servo primaryMotor, Servo secondaryMotor)
{
    int primaryPower = map(powerMultiplier*pid.Compute(), 0f, PIDmax, 30f, 130f);
    int secondaryPower = map(powerMultiplier/pid.Compute(), 0f, 1/PIDmax, 30f, 130f);
    primaryMotor.write(primaryPower);
    secondaryMotor.write(secondaryPower);
}

void applyYaw


void loop() 
{
    applyMotorPower(pitchPID, frontMotor, backMotor);
    applyMotorPower(rollPid, frontMotor, backMotor);
    
}