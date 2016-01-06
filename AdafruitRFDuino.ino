#include <Wire.h>

//ref https://github.com/damienromito/RFDuino-adafruit-lSM9DS0
//need to fix "return value of getEvent method" and so on ( now is 2015/12/10 )

//need Adafruit_LSM9DS0 sensorapi https://github.com/adafruit/Adafruit_LSM9DS0_Library/tree/master/examples
//need runningMedian  http://playground.arduino.cc/Main/RunningMedian
//need RFduino https://github.com/RFduino/RFduino/tree/master/libraries/RFduinoBLE

#include <Adafruit_LSM9DS0_RFDUINO.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <RFduinoBLE.h>
#include "RunningMedian.h"

// i2c
Adafruit_LSM9DS0_RFDUINO lsm = Adafruit_LSM9DS0_RFDUINO(1000);

// You can also use software SPI
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(13, 12, 11, 10, 9);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(10, 9);

int State = 0;

float Offset_ax, Offset_ay, Offset_az;
float Offset_gx, Offset_gy, Offset_gz;
float Offset_mx, Offset_my, Offset_mz;

int MedianV = 10;

float Ax , Ay, Az;
float Gx , Gy, Gz;
float Mx , My, Mz;

float BAx = 0 , BAy = 0, BAz = 0;
float BGx = 0 , BGy = 0, BGz = 0;
float BMx = 0 , BMy = 0, BMz = 0;


void setupSensor()
{
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
}

void setup() 
{
  while (!Serial); // flora & leonardo
  
  Serial.begin(9600);
  Serial.println("LSM raw read demo");
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS0. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS0 9DOF");
  Serial.println("");
  Serial.println("");
  RFduinoBLE.begin();
}

void loop() 
{
  if (State == 0)
  {
    Serial.println("\nReadingsensorsforfirsttime");
    SetData();
    State++;
    delay(1000);
  }

  if (State == 1) {
    Serial.println("\nCalculating offsets");
    SetOffset();
    State++;
    delay(1000);
  }

  if (State == 2)
  {
    SetData();
    Ax -= Offset_ax;
    Ay -= Offset_ay;
    Az -= Offset_az;

    Gx -= Offset_gx;
    Gy -= Offset_gy;
    Gz -= Offset_gz;

    Mx -= Offset_mx;
    My -= Offset_my;
    Mz -= Offset_mz;

    //define Sensor Error
    //Smoothing
    float deadZoneAcc = 0.07;
    if (abs(Ax - BAx) < deadZoneAcc) Ax = BAx;
    if (abs(Ay - BAy) < deadZoneAcc) Ay = BAy;
    if (abs(Az - BAz) < deadZoneAcc) Az = BAz;

    float deadZoneGyro = 0.7;
    if (abs(Gx - BGx) < deadZoneGyro) Gx = BGx;
    if (abs(Gy - BGy) < deadZoneGyro) Gy = BGy;
    if (abs(Gz - BGz) < deadZoneGyro) Gz = BGz;

    float deadZoneM = 0.7;
    if (abs(Mx - BMx) < deadZoneM) Mx = BMx;
    if (abs(My - BMy) < deadZoneM) My = BMy;
    if (abs(Mz - BMz) < deadZoneM) Mz = BMz;

    if (abs(Ax) < deadZoneAcc) Ax = 0;
    if (abs(Ay) < deadZoneAcc) Ay = 0;
    if (abs(Az) < deadZoneAcc) Az = 0;

    if (abs(Gx) < deadZoneGyro) Gx = 0;
    if (abs(Gy) < deadZoneGyro) Gy = 0;
    if (abs(Gz) < deadZoneGyro) Gz = 0;

    if (abs(Mx) < deadZoneM) Mx = 0;
    if (abs(My) < deadZoneM) My = 0;
    if (abs(Mz) < deadZoneM) Mz = 0;
    Serial.print("A:"); Serial.print(Ax); Serial.print(","); Serial.print(Ay); Serial.print(","); Serial.print(Az); Serial.print("_"); // m/s^2
    Serial.print("M:"); Serial.print(Mx); Serial.print(","); Serial.print(My); Serial.print(","); Serial.print(Mz); Serial.print("_"); // gauss
    Serial.print("G:"); Serial.print(Gx); Serial.print(","); Serial.print(Gy); Serial.print(","); Serial.println(Gz); //dps
    BAx = Ax;
    BAy = Ay;
    BAz = Az;

    BGx = Gx;
    BGy = Gy;
    BGz = Gz;

    BMx = Gx;
    BMy = Gy;
    BMz = Gz;
  }
 
  
}

void SetOffset()
{
  SetData();

  Offset_ax = Ax - 0;
  Offset_ay = Ay - 0;
  Offset_az = Az - 0;

  Offset_gx = Gx - 0;
  Offset_gy = Gy - 0;
  Offset_gz = Gz - 0;

  Offset_mx = Mx - 0;
  Offset_my = My - 0;
  Offset_mz = Mz - 0;
}

void SetData()
{
  RunningMedian Rax = RunningMedian(MedianV), Ray  = RunningMedian(MedianV), Raz  = RunningMedian(MedianV);
  RunningMedian Rgx = RunningMedian(MedianV), Rgy  = RunningMedian(MedianV), Rgz  = RunningMedian(MedianV);
  RunningMedian Rmx = RunningMedian(MedianV), Rmy  = RunningMedian(MedianV), Rmz  = RunningMedian(MedianV);
  
  sensors_event_t accel, mag, gyro, temp;
  for (int i = 0; i < MedianV; i++)
  {
    lsm.getEvent(&accel, &mag, &gyro, &temp);

    float gx = gyro.gyro.x;
    float gy = gyro.gyro.y;
    float gz = gyro.gyro.z;

    float ax = accel.acceleration.x;
    float ay = accel.acceleration.y;
    float az = accel.acceleration.z;

    float mx = mag.magnetic.x;
    float my = mag.magnetic.y;
    float mz = mag.magnetic.z;

    Rgx.add(gx);
    Rgy.add(gy);
    Rgz.add(gz);

    Rax.add(ax);
    Ray.add(ay);
    Raz.add(az);

    Rmx.add(mx);
    Rmy.add(my);
    Rmz.add(mz);

    i++;
    delay(2);
  }

  //Exclusion of outlier
  Gx = Rgx.getAverage(3);
  Gy = Rgy.getAverage(3);
  Gz = Rgz.getAverage(3);

  Mx = Rmx.getAverage(3);
  My = Rmy.getAverage(3);
  Mz = Rmz.getAverage(3);

  Ax = Rax.getAverage(3);
  Ay = Ray.getAverage(3);
  Az = Raz.getAverage(3);
}
