#include <MadgwickAHRS.h>
#include <PID_AutoTune_v0.h>
#include <Servo.h>
#include <PID_v1.h>

/* Authors: John Petryk and Henryk Viana
 * Arduino code for running the drone
 */

//pins
Servo frontMotor
Servo backMotor
Servo rightMotor
Servo leftMotor

Madgwick filter;

//pid perameters
float pitchSetpoint, pitchInput, pitchOutput; // pitch PID perameters 
float yawSetpoint, yawInput, yawOutput; // yaw PID perameters
float rollSetpoint, rollInput, rollOutput; // roll PID perameters
float altitudeSetpoint, altitudeInput, altitudeOutput; // altitude PID perameters

//PID objects - & symbols for global varables
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, 1, 0, 0, DIRECT);
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, 1, 0, 0, DIRECT);
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, 1, 0, 0, DIRECT);

float PIDmin = 1;
float PIDmax = sqrt(2);

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
}


void applyMotorPower(PID pid, Servo primaryMotor, Servo secondaryMotor)
{
    int primaryPower = map(powerMultiplier*pid.Compute(), 0, sqrt(2), 30, 130);
    int secondaryPower = map(powerMultiplier/pid.Compute(), 0, 1/sqrt(2), 30, 130);
    primaryMotor.write(primaryPower);
    secondaryMotor.write(seccondaryPower)
}



void loop() 
{
    //TODO: add gyro and acclerometer code

    // These will return ratios of how much to differentiate the power levels in different motors
    pitchPID.Compute();
    yawPID.Compute();
    rollPID.Compute();
}