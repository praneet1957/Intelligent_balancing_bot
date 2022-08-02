//The sample code for driving one way motor encoder

#include <TimerOne.h>
const byte interruptPin = 2;//A pin -> the interrupt pin 0
const byte encoder0pinA = 6;//B pin -> the digital pin 3

boolean Direction;//the rotation direction

float currentTime_enc  = 0;
float previousTime_enc = 0;

float wheel_speed = 0;


void EncoderInit()
{
  Direction = true;//default -> Forward
  pinMode(encoder0pinA,INPUT);

}

float readEncoder()
{
  float dt  = 0;
  float dte = 0;
  currentTime_enc  = millis();  
  previousTime_enc = millis();
  int start = 5;
  float start_time,end_time;
  
  while(dt<1){
  currentTime_enc =  millis();
   
  int Lstate = digitalRead(encoder0pinA);
  if (Lstate==LOW && start==5 ){
    start = 0;    }
  
  else if(Lstate == HIGH && start==0){
    start = 1;
    start_time = millis();
    //Serial.print(start_time);
     }

  else if(Lstate == LOW && start==1){
    end_time = millis();
    dte = (end_time - start_time)/1000;
   
    wheel_speed = 3.1415*0.065/(40*dte);
    break;
    //return wheel_speed;
    }
   
  dt =  (currentTime_enc - previousTime_enc)/1000;
    }
    
 if (dt>=1){
  wheel_speed=0;
 }

 return wheel_speed;
}

  
 


void setup()
{
  Serial.begin(1200); //Initialize the serial port
  EncoderInit();      //Initialize the module
//  attachInterrupt(digitalPinToInterrupt(interruptPin), readEncoder, CHANGE);
//  pinMode(interruptPin, OUTPUT);
//  Timer1.initialize(1000000);
//  Timer1.attachInterrupt(readEncoder);
//  digitalWrite(interruptPin,LOW);
}

void loop()
{

  
  wheel_speed = readEncoder();
 

  Serial.print("vel:");
  Serial.println(wheel_speed);

}
