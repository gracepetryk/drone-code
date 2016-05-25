#include <Servo.h>

Servo frontMotor;
Servo backMotor;
Servo rightMotor;
Servo leftMotor;



void setup() {
	  frontMotor.attach(4);
    rightMotor.attach(5);
    backMotor.attach(6);
    leftMotor.attach(7);
    
    frontMotor.write(30);
    rightMotor.write(30);
    backMotor.write(30);
    leftMotor.write(30);

    delay(5000);
}

void loop() {
	frontMotor.write(100);
	delay(5000);
  frontMotor.write(30);
	rightMotor.write(100);
	delay(5000);
  rightMotor.write(30);
	backMotor.write(100);
	delay(5000);
  backMotor.write(30);
	leftMotor.write(100);
	delay(5000);
  leftMotor.write(30);
}
