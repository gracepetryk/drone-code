#include <PID_v1.h>
/* Authors: John Petryk and Henryk Viana
 * Arduino code for running the drone
 */

//pins
int frontMotorPin = 1;
int backMotorPin = 2;
int rightMotorPin = 3;
int leftMotorPin = 4;

//pid perameters
double pitchSetpoint, pitchInput, pitchOutput; // pitch PID perameters 
double yawSetpoint, yawInput, yawOutput; // yaw PID perameters
double rollSetpoint, rollInput, rollOutput; // roll PID perameters
double altitudeSetpoint, altitudeInput, altitudeOutput; // altitude PID perameters

//PID objects - & symbols for global varables
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, 1, 1, 1, DIRECT);
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, 1, 1, 1, DIRECT);
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, 1, 1, 1, DIRECT);

double PIDmin = 0.5;
double PIDmax = 1.5;

double powerMultiplier = 0;

void setup() 
{
    // initialize pins
    pinMode(frontMotorPin, OUTPUT);
    pinMode(backMotorPin, OUTPUT);
    pinMode(rightMotorPin, OUTPUT);
    pinMode(leftMotorPin, OUTPUT);

    // intialize PID's
    pitchPID.SetMode(AUTOMATIC);
    yawPID.SetMode(AUTOMATIC);
    rollPID.SetMode(AUTOMATIC);

    //set PID limits
    pitchPID.SetOutputLimits(PIDmin, PIDmax);
    yawPID.SetOutputLimits(PIDmin, PIDmax);
    rollPID.SetOutputLimits(PIDmin, PIDmax);
}

void loop() 
{
    //TODO: add gyro and acclerometer code

    // These will return ratios of how much to differentiate the power levels in different motors
    pitchPID.Compute();
    yawPID.Compute();
    rollPID.Compute();
}


