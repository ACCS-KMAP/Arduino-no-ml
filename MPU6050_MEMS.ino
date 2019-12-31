#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
#define PI 3.1415926535897932384626433832795
int16_t ax, ay, az;
int16_t gx, gy, gz;
float yax, yay, yaz, ygx, ygy, ygz;
int16_t f_cut = 5;
int16_t n = 1;
float gxPrevious,gyPrevious,gzPrevious;
float ygx_prev, ygy_prev , ygz_prev , yax_prev ,yay_prev ,yaz_prev;
// roll,roll_prev,pitch,pitch_prev,yaw;
float AccX, AccY, AccZ , rms_a;
float GyX, GyY, GyZ , rms_gy;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
//float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
int prev_n = -100;


void setup() {
  Serial.begin(19200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  /*
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  */
  // Call this function if you need to get the IMU error values for your module
  //calculate_IMU_error();
  delay(20);
}
void loop() {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read())/ 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read())/ 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read())/ 16384.0; // Z-axis value
  rms_a=pow((pow(AccX,2)+pow(AccY,2)+pow(AccZ,2)),0.5);

  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); 

  GyX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  
  /*Serial.print("AccX: ");
  Serial.print(AccX);
  Serial.print("\tAccY: ");
  Serial.print(AccY);
  Serial.print("\tAccZ: ");
  Serial.print(AccZ);*/

 
  /*Serial.print("\tGyX: ");
  Serial.print(GyX);
  Serial.print("\tGyY: ");
  Serial.print(GyY);
  Serial.print("\tGyZ: ");
  Serial.print(GyZ);*/




  lowpassfilter(AccX,AccY,AccZ,n,f_cut);
  Serial.print("\trms_a: ");
  Serial.print(rms_a*100);
  /*if(n == 1)
  {
    mini_n = rms_a*100;
    maxi_n = rms_a*100;
    mini_index = 1;
    maxi_index = 1;
  }*/
  
  if(rms_a*100 > 150 and n-prev_n < 30)
    Serial.print("\nFall Detected");
  if(rms_a*100 < 80 and n-prev_n < 30)
    Serial.print("\nFall Detected");
  if(rms_a*100 < 80 and n-prev_n < 30)
    prev_n = n;

  
    
  highpassfilter(GyX,GyY,GyZ,n,f_cut);
  rms_gy=pow((pow(GyX,2)+pow(GyY,2)+pow(GyZ,2)),0.5);
  Serial.print("\trms_gy: ");
  Serial.print(rms_gy);
  Serial.println();

    //Serial.print("yaw= ");Serial.print(abs(yaw));Serial.print("\t");
  delay(20);
}

void lowpassfilter(float ax,float ay,float az,int16_t n,int16_t f_cut)
{
  float dT = 0.01;  //time in seconds
  float Tau= 1/(2*3.1457*f_cut);                   //f_cut = 5
  float alpha = Tau/(Tau+dT);                //do not change this line

  if(n == 1)
  {
    yax = (1-alpha)*ax ;
    yay = (1-alpha)*ay ;
    yaz = (1-alpha)*az ;
  }
  else
  {
    yax = (1-alpha)*ax + alpha*yax_prev;
    yay = (1-alpha)*ay + alpha*yay_prev;
    yaz = (1-alpha)*az + alpha*yaz_prev;  
  }  
  yax_prev = yax;
  yay_prev = yay;
  yaz_prev = yaz;
//Serial.print("yax ");Serial.print(yax/1000);Serial.print("\t");
//Serial.println();
//Serial.print("yay ");Serial.print(yay/1000);Serial.print("\t");
//Serial.println();
//Serial.print("yaz ");Serial.print((yaz/1000));Serial.print("\t");
//Serial.println();
//Serial.println();
    

}

void highpassfilter(float gx,float gy,float gz,int16_t n,int16_t f_cut)
{
  
  float dT = 0.01;  //time in seconds
  float Tau= 1/(2*3.1457*f_cut);                   //f_cut = 5
  float alpha = Tau/(Tau+dT);                //do not change this line
 
  if(n == 1)
  {
    ygx = (1-alpha)*gx ;
    ygy = (1-alpha)*gy ;
    ygz = (1-alpha)*gz ;

  }
  else
  {
    ygx = (1-alpha)*ygx_prev + (1-alpha)*(gx - gxPrevious);
    ygy = (1-alpha)*ygy_prev + (1-alpha)*(gy - gyPrevious);
    ygz = (1-alpha)*ygz_prev + (1-alpha)*(gz - gzPrevious);
  }
  gxPrevious = gx;
  gyPrevious = gy;
  gzPrevious = gz;

  ygx_prev = ygx;
  ygy_prev = ygy;
  ygz_prev = ygz;
  
/*Serial.print("ygx= ");Serial.print(ygx);Serial.print("\t");
//Serial.println();
Serial.print("ygy= ");Serial.print(ygy);Serial.print("\t");
//Serial.println();
Serial.print("ygz= ");Serial.print(ygz);Serial.print("\t");
Serial.println();*/


}
