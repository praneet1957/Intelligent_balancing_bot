
#include "I2Cdev.h"
#include "MPU6050.h"


#include "Wire.h"


MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

const int MPU_addr=0x68;                   // I2C address of the MPU-6050
float AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;        
const byte inter_pin = 11;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float currentTime, elapsedTime, previousTime;

int16_t ax, ay, az;
int16_t gx, gy, gz;
float accel_divide, gyro_divide;

float IMU(){
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
  AcX = (float)ax*9.8/accel_divide;
  AcY = (float)ay*9.8/accel_divide;
  AcZ = (float)az*9.8/accel_divide;
  GyX = (float)gx/gyro_divide;
  GyY = (float)gy/gyro_divide;
  GyZ = (float)gz/gyro_divide;
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

  return pitch ;

}





void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)

    Wire.begin();


    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(19200);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");


    // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
    Wire.write(0x10);
    accel_divide = 4096;//Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission(true);
    // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
    Wire.write(0x10);      
    gyro_divide = 32.8; // Set the register bits as 00010000 (1000deg/s full scale)
    Wire.endTransmission(true);

    
    // use the code below to change accel/gyro offset values
   
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
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
   
   
    delay(20);

}

void loop() {
    // read raw accel/gyro measurements from device
    //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    

    roll = IMU();
    Serial.print("roll");
    Serial.println(roll);


}
