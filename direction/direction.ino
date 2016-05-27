#include <Servo.h>

Servo frontMotor;
Servo backMotor;
Servo rightMotor;
Servo leftMotor;



void setup() {
	  frontMotor.attach(5);
    rightMotor.attach(6);
    backMotor.attach(9);
    leftMotor.attach(10);
    
    frontMotor.write(30);
    rightMotor.write(30);
    backMotor.write(30);
    leftMotor.write(30);

    delay(5000);
}

void loop() {
	frontMotor.write(50);
	delay(5000);
  frontMotor.write(30);
	rightMotor.write(50);
	delay(5000);
  rightMotor.write(30);
	backMotor.write(50);
	delay(5000);
  backMotor.write(30);
	leftMotor.write(50);
	delay(5000);
  leftMotor.write(30);
}
