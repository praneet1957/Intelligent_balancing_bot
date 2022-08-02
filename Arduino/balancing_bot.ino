#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <TimerOne.h>
int i = 1;
MPU6050 accelgyro;
#include <AFMotor.h>

AF_DCMotor motor1(2, MOTOR12_64KHZ);
AF_DCMotor motor2(3, MOTOR12_64KHZ);


#define max_pwm  255
#define min_pwm  0

float Kp =55 , Kd =0.5, Ki =0;
float int_error = 0;
float desired   = 0;
double radius   = 0.065/2;
float N = 20;
float prev_time, curr_time;
float PWM_prev;



//IMU variables
const int MPU_addr=0x68;                   // I2C address of the MPU-6050
float AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;        
const byte inter_pin = 2;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float currentTime, elapsedTime, previousTime;

int16_t ax, ay, az;
int16_t gx, gy, gz;
float accel_divide, gyro_divide;


//
//
//
//encoder variables
const byte interruptPin = 2;//
const byte encoder0pinA = 6;//
const byte encoder0pinB = 5;
boolean Direction;//the rotation direction

float currentTime_enc = 0;
float previousTime_enc = 0;
float wheel_speed = 0;


//
//
//
//
////motor variables
//const byte motorpin1 = 3;
//const byte motorpin2 = 11;
//
//
//
//
//
void init_encoder(){
  Direction = true;//default -> Forward
  pinMode(encoder0pinA,INPUT);
  pinMode(encoder0pinB,INPUT);
}

//
//
//
//
float readEncoder(int encoderpin)
{
  float dt = 0;
  float dte = 0;
  currentTime_enc =  millis();  
  previousTime_enc = millis();
  int start = 5;
  float start_time,end_time;
  
  while(dt<1){
  currentTime_enc =  millis();
   
  int Lstate = digitalRead(encoderpin);
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
//
//








////IMU code for measuring theta
void IMU(float arr[]){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers  String AX = String(mpu6050.getAccX());
  
//  AcX=((Wire.read()<<8|Wire.read())/16384);  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
//  AcY=((Wire.read()<<8|Wire.read())/16384);  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
//  AcZ=((Wire.read()<<8|Wire.read())/16384);  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
//  Tmp=  Wire.read()<<8|Wire.read();          // 0x41 (TEMP_OUT_H)   & 0x42 (TEMP_OUT_L)
//  GyX=(Wire.read()<<8|Wire.read())/131;      // 0x43 (GYRO_XOUT_H)  & 0x44 (GYRO_XOUT_L)
//  GyY=(Wire.read()<<8|Wire.read())/131;      // 0x45 (GYRO_YOUT_H)  & 0x46 (GYRO_YOUT_L)
//  GyZ=(Wire.read()<<8|Wire.read())/131;      // 0x47 (GYRO_ZOUT_H)  & 0x48 (GYRO_ZOUT_L)


  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  AcX = (float)ax*9.8/accel_divide ;
  AcY = (float)ay*9.8/accel_divide ;
  AcZ = (float)az*9.8/accel_divide ;
  GyX = (float)gx/gyro_divide ;
  GyY = (float)gy/gyro_divide ;
  GyZ = (float)gz/gyro_divide ;
  accAngleX = (atan(AcY / sqrt(pow(AcX, 2) + pow(AcZ, 2))) * 180 / PI); 
  accAngleY = asin(-1 * AcX / sqrt(pow(AcX, 2) + pow(AcZ, 2))) * 180 / PI;
  //Serial.println(accAngleX);
  // === Read gyroscope data === //
         
  currentTime =  millis();           
  elapsedTime = (currentTime - previousTime) / 1000; 
  previousTime = currentTime; 

  gyroAngleX = gyroAngleX + GyX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyY * elapsedTime;
  yaw        =  yaw + GyZ * elapsedTime;

  roll  = 0.04 * gyroAngleX + 0.96 * accAngleX;
  pitch = 0.04 * gyroAngleY + 0.96 * accAngleY;


  arr[0] = pitch;
  arr[1] = GyY;


}








float PWM_angle(float angle, float gyrox){
  float error;
  float pid;
  
  error = desired - angle;
  curr_time = millis();
  
  //int_error = int_error + error*(curr_time-prev_time)/1000;
  pid = Kp*error + Kd*gyrox + Ki*int_error;
  prev_time = curr_time;
  return pid;
}






void setup() {
  // intializing IMU
  Wire.begin();
  Serial.begin(19200);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);
  accel_divide = 4096;        //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);      
  gyro_divide = 32.8;          // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
 
  Serial.println("Updating internal sensor offsets...");

  int ax_off = accelgyro.getXAccelOffset();
  int ay_off = accelgyro.getYAccelOffset();
  int az_off = accelgyro.getZAccelOffset();
  int gx_off = accelgyro.getXGyroOffset();
  int gy_off = accelgyro.getXGyroOffset();
  int gz_off = accelgyro.getXGyroOffset();
  
  accelgyro.setXAccelOffset(ax_off);
  accelgyro.setYAccelOffset(ay_off);
  accelgyro.setZAccelOffset(az_off);
  accelgyro.setXGyroOffset(gx_off);
  accelgyro.setYGyroOffset(gy_off);
  accelgyro.setZGyroOffset(gz_off);
  Serial.print("\n");


//  init_encoder();

}




//
void loop() {
  // put your main code here, to run repeatedly:

    float angle = 0;
    float gyrox = 0;
    float delta_PWM = 0;
    float imu_data[2] ;
    int PWM_value = 0;
    float PWM_output = 0;
//
  IMU(imu_data);
//  gyrox = 0.5*readEncoder(encoder0pinA);
//  gyrox = gyrox + 0.5*readEncoder(encoder0pinB);

  angle = imu_data[0];
  gy = imu_data[1];
  delta_PWM = PWM_angle(angle,gy);
  PWM_output =  PWM_prev + delta_PWM;
  
  if(abs(PWM_output) > max_pwm){
    PWM_value = max_pwm;
  }

  else if(abs(PWM_output) < min_pwm){
    PWM_value = min_pwm;
  }

  else{
    PWM_value = (int) abs(PWM_output);
  }
    
    motor1.setSpeed(PWM_value);
    motor2.setSpeed(PWM_value);
  
  if (PWM_output>=0){
    motor1.run(FORWARD);
//    motorA.setSpeed(0);
    motor2.run(FORWARD);
//    motorB.setSpeed(0);
   
   Serial.println("ford");
  }
  
  else{
    motor1.run(BACKWARD);
//    motorA.setSpeed(0);
    motor2.run(BACKWARD);


    
    Serial.println("forward");
  }

  Serial.print("PWM_value");
  Serial.print(PWM_value);
  Serial.print("PWM_output");
  Serial.print(PWM_output);
  Serial.print("pitch");
  Serial.print(angle);
  Serial.print("gyro");
  Serial.println(gy);
  
 }


 
