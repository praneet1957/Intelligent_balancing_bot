#include <AFMotor.h>

AF_DCMotor motor1(3, MOTOR12_64KHZ);
AF_DCMotor motor2(2, MOTOR12_64KHZ);
void setup(){
  Serial.begin(19200);
}



void loop(){
//
//motor1.run(FORWARD);
//motor1.setSpeed(00);

//
//delay(2);
motor1.setSpeed(10);
motor1.run(FORWARD);
}
 
