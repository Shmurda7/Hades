#include <Arduino.h>
#include <MPU9250.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP085.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include "KalmanFilter.h"
#include "eigen.h"      // Calls main Eigen matrix class library
#include <Eigen/LU>   
#ifdef abs
#undef abs
#endif
using namespace Eigen;

MPU9250 imu(Wire, 0x68);
Adafruit_BMP085 bmp;
TinyGPSPlus gps;

int state;
bool hasLaunched = false;
bool haspressed = false;
double gravity = 9.81;

float roll_filtered = 0, pitch_filtered = 0, yaw_filtered = 0;

// ------------ IMU Variables -------------- //
// Accelerometer
double xaccel = 0, yaccel = 0, zaccel = 0;
// Gyroscope
double roll_rate = 0, pitch_rate = 0, yaw_rate = 0;
double roll_angle = 0, pitch_angle = 0, yaw_angle = 0;
// Magnometer
double xmag = 0, ymag = 0, zmag = 0;
// Temperature
float i_temp;

// ------------ BMP Variables -------------- //
// Pressure
float pres;
// Temperature
float b_temp;
float rawpres;
float rawtemp;
float currentaltitude;

// GPS Variables
static const int rxPin = 0;
static const int txPin = 1;
SoftwareSerial ss(rxPin, txPin);

// ----------- Hardware Pins --------------- //
#define BUTTON_PIN 23
#define BUZZER_PIN 41
#define RED_LED_PIN 6
#define WHITE_LED_PIN 8
#define GREEN_LED_PIN 4
#define PYRO_1_PIN 56
#define PYRO_2_PIN 57

// options for sequencing
int r = 0;
int w = 0;
int g = 0;
int buz = 0;

//Continuity Data/LEDs
/*
int cnt1 = 9;
int cnt2 = 10;
int p1 = 11; //led to indicate continuity
int p2 = 12; //led to indicate continuity
int read1, read2;
*/

// ------------- Math Portion --------------- //
// Altitude
float initialaltitude;
float sealevelpres = 101325; // Pa // chage for pressure here
float maxaltitude = 0;
double temp;

// Filters Gains
int lpfGyr = 0.7; // need to tune low pass filter
int lpfAcc = 0.9;
int lpfMag = 0.4;
double alpha = 0.05f;

float euler_roll_rate = 0, euler_pitch_rate = 0, euler_yaw_rate = 0;

// Complimentary filter variables
double accel_roll_angle = 0, accel_pitch_angle = 0, accel_yaw_angle;
double mag_roll_angle = 0, mag_pitch_angle = 0, mag_yaw_angle = 0;
double comp_roll_angle = 0, comp_pitch_angle = 0, comp_yaw_angle = 0;

double comp_roll_angle_dos = 0, comp_pitch_angle_dos = 0;

// Bias
double bias_roll_rate, bias_pitch_rate, bias_yaw_rate;

// Time
float elapsedtime{0.0f};
float currenttime{0.0f};
float previoustime{0.0f};
/*
KalmanFilter kalmanRoll(0, 1, 0.01, 0.1);
KalmanFilter kalmanPitch(0, 1, 0.01, 0.1);
KalmanFilter kalmanYaw(0, 1, 0.01, 0.1);
*/
File myFile;

// put function declarations here:
void flight();
void launchdetect();
void deploy();
void protocol();
void calibration();
void startup();
void datalogging();
//void kalmanFilterUpdate(Eigen::MatrixXf& xk, Eigen::MatrixXf& P,const Eigen::MatrixXf& A, const Eigen::MatrixXf& H,const Eigen::MatrixXf& Q, const Eigen::MatrixXf& R,const Eigen::MatrixXf& zk);

void setup() {
  
  Serial.begin(115200);

  // Pin out for States
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(WHITE_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PYRO_1_PIN, OUTPUT);
  pinMode(PYRO_2_PIN, OUTPUT);
  //other pins
  /*
  pinMode(cnt1, OUTPUT);
  pinMode(cnt2, OUTPUT);
  pinMode(p1, OUTPUT);
  pinMode(p2, OUTPUT);
  */

 // ----------- Sensor Startup ------------ //
  imu.begin();
  // setting the accelerometer full scale range to +/-8G
  imu.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-250 deg/s
  imu.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  imu.setSrd(19);
  // Barometric Pressure Sensor
  bmp.begin();
  initialaltitude = bmp.readAltitude();
  Serial.println("Initial Alt: ");
  Serial.println(initialaltitude);

  ss.begin(9600);

    // Data Logging Initilization
  if (!SD.begin(BUILTIN_SDCARD))
  {
    Serial.println("SD card initialization failed!");
    while (1) {}
  }
  Serial.println("SD card initialized.");
  // Open the file
  myFile = SD.open("Jan_Eleventh_Test_Data.txt", FILE_WRITE);
  if (myFile)
  {
    myFile.println(" ---------- Flight Data -----------");
    myFile.close();
  }
  else
  {
    Serial.println("Error opening data file!");
  }

  startup();
  //calibration();
  protocol();
}

void loop() {
  currenttime = millis();
  elapsedtime = (currenttime - previoustime) / 1000;

  imu.readSensor();
  xaccel = lpfAcc * xaccel + (1 - lpfAcc) * imu.getAccelX_mss();
  yaccel = lpfAcc * yaccel + (1 - lpfAcc) * imu.getAccelY_mss();
  zaccel = lpfAcc * zaccel + (1 - lpfAcc) * imu.getAccelZ_mss();
  roll_rate = lpfGyr * roll_rate + (1 - lpfGyr) * imu.getGyroX_rads();
  pitch_rate = lpfGyr * pitch_rate + (1 - lpfGyr) * imu.getGyroY_rads();
  yaw_rate = lpfGyr * yaw_rate + (1 - lpfGyr) * imu.getGyroZ_rads();
  xmag = lpfMag * xmag + (1 - lpfMag) * imu.getMagX_uT();
  ymag = lpfMag * zmag + (1 - lpfMag) * imu.getMagY_uT();
  zmag = lpfMag * zmag + (1 - lpfMag) * imu.getMagZ_uT();
  i_temp = imu.getTemperature_C();

  //right way according to phils lab
  accel_roll_angle = atanf(yaccel / zaccel);
  accel_pitch_angle = asinf(yaccel / gravity);
  //might be wrong
  /*
  accel_roll_angle = atan2(-xaccel, sqrt(pow(yaccel, 2) + pow(zaccel, 2))) * RAD_TO_DEG;
  accel_pitch_angle = atan2(yaccel, sqrt(pow(xaccel, 2) + pow(zaccel, 2))) * RAD_TO_DEG;
  accel_yaw_angle = atan2((sqrt(pow(xaccel, 2) + pow(yaccel, 2))), zaccel) * RAD_TO_DEG;
  */

  mag_roll_angle = atan2(ymag, zmag);
  mag_pitch_angle = atan2(xmag, zmag);
  mag_yaw_angle = atan2(ymag, xmag);

  roll_angle += (roll_rate * elapsedtime);
  pitch_angle += (pitch_rate * elapsedtime);
  yaw_angle += (yaw_rate  * elapsedtime);

  // https://control.asu.edu/Classes/MMAE441/Aircraft/441Lecture9.pdf

  euler_roll_rate = roll_rate + tanf(comp_roll_angle)*sinf(comp_pitch_angle)*pitch_rate + cosf(comp_pitch_angle)*tanf(comp_roll_angle)*yaw_rate;
  euler_pitch_rate = 0 + cosf(comp_pitch_angle) - sinf(comp_pitch_angle);
  euler_yaw_rate = 0 + sinf(comp_pitch_angle)*(1/cosf(comp_roll_angle))*pitch_rate + cosf(comp_pitch_angle)*(1/cosf(comp_roll_angle))*yaw_rate;

  // Complimentary Filter phils lab
  //flipped from original bc i had implemented it wrong in the first place 
  comp_roll_angle = alpha * mag_roll_angle + (1 - alpha) * (comp_roll_angle + roll_rate * (elapsedtime));
  comp_pitch_angle = alpha * mag_pitch_angle + (1 - alpha) *(comp_pitch_angle + pitch_rate * (elapsedtime));
  comp_yaw_angle = alpha * mag_yaw_angle + (1 - alpha) * (comp_yaw_angle + yaw_rate * (elapsedtime));
  // So a slight different between the roll and pitch angles theres a -7 degs for roll and -6 deg for pitch, double check 
  //with testing to see which is more true and then thats what i'll choose or can change the gain
  comp_roll_angle_dos = alpha * accel_roll_angle + (1 - alpha) * (comp_roll_angle + roll_rate * (elapsedtime));
  comp_pitch_angle_dos = alpha * accel_pitch_angle + (1 - alpha) *(comp_pitch_angle + pitch_rate * (elapsedtime));
  
  //Original Comp Filter 
  /*
  comp_roll_angle = (1 - alpha) * mag_roll_angle + alpha * (roll_angle + roll_rate * elapsedtime);
  comp_pitch_angle = (1 - alpha) * mag_pitch_angle + alpha *(pitch_angle + pitch_rate * elapsedtime);
  comp_yaw_angle = (1 - alpha) * mag_yaw_angle + alpha * (yaw_angle + yaw_rate * elapsedtime);
  */  
  //complimentary filter for position with x, and xdot from accel
  //then use gps similar to mag above, but make sure to convert the position unit stuff
  //so essentially it'll be just converting x and y position to lat and long

  //ok section for combining GPS and accelerometer data for postion and velocity
  // I don't think my gps and can altitude

  // Kalman Filter
   // Update Kalman filter with accelerometer measurements
   /*
  roll_filtered = kalmanRoll.update(accel_roll_angle* RAD_TO_DEG);
  pitch_filtered = kalmanPitch.update(accel_pitch_angle* RAD_TO_DEG);
  yaw_filtered = kalmanYaw.update(mag_yaw_angle* RAD_TO_DEG);
  */
  //gps_pos_x
  //gps_pos_y
  //gps_pos_z


  // BMP
  b_temp = bmp.readTemperature();
  currentaltitude = bmp.readAltitude(sealevelpres)/3.281; // ft
  //sealevelpres = bmp.readSealevelPressure()/6895; // psi
  pres = bmp.readPressure()/6895; // psi
  rawpres = bmp.readRawPressure();
  rawtemp = bmp.readRawTemperature();
  //temperature from imu and baro
  temp = (((b_temp + i_temp)/2) * (9/5)) + 32; // F

  byte gpsData = ss.read();
  gps.encode(gpsData);

  float buttonpres = digitalRead(BUTTON_PIN);

  if (buttonpres && !haspressed){ //this only data logs the 1 time you press the button
    datalogging();
    Serial.println("---------");
    Serial.println("---------");
    Serial.println("logging!");
    Serial.println("---------");
    Serial.println("---------");
    haspressed = true;
  }

  //flight();
  launchdetect();

  if (hasLaunched) {
    flight();
  }
  
  previoustime = currenttime;
}

void datalogging(){
  // Data Logging Part
  myFile = SD.open("Jan_Eleventh_Test_Data.txt", FILE_WRITE);
  if (myFile)
  {
    // Data Stuff
    // s
    myFile.print(" time: ");
    myFile.print(currenttime/1000, 3);
    myFile.print(",");
    myFile.print(" Unfiltered Data: ");
    // m/s2
    myFile.print(" ax: ");
    myFile.print(imu.getAccelBiasX_mss());
    myFile.print(",");
    myFile.print(" ay: ");
    myFile.print(imu.getAccelBiasX_mss());
    myFile.print(",");
    myFile.print(" az: ");
    myFile.print(imu.getAccelBiasX_mss());
    myFile.print(",");
    // rad/s
    myFile.print(" gx: ");
    myFile.print(imu.getGyroBiasX_rads());
    myFile.print(",");
    myFile.print(" gy: ");
    myFile.print(imu.getGyroBiasY_rads());
    myFile.print(",");
    myFile.print(" gz: ");
    myFile.print(imu.getGyroBiasZ_rads());
    myFile.print(",");
    // uT
    myFile.print(" mx: ");
    myFile.print(imu.getMagX_uT());
    myFile.print(",");
    myFile.print(" my: ");
    myFile.print(imu.getMagY_uT());
    myFile.print(",");
    myFile.print(" mz: ");
    myFile.print(imu.getMagZ_uT());
    myFile.print(",");
    // rad
    myFile.print(" roll: ");
    myFile.print(roll_angle);
    myFile.print(",");
    myFile.print(" pitch: ");
    myFile.print(pitch_angle);
    myFile.print(",");
    myFile.print(" yaw: ");
    myFile.print(yaw_angle);
    myFile.print(",");
    // Celcius
    myFile.print("IMU_Temp: ");
    myFile.print(i_temp);
    myFile.print(",");
    myFile.print("Baro_Temp: ");
    myFile.print(b_temp);
    myFile.print(",");
    // psi
    myFile.print("Pres: ");
    myFile.print(pres);
    myFile.print(",");
    // Filtered Data
    myFile.print(" Filtered Data: ");
    // m/s2
    myFile.print(" ax: ");
    myFile.print(xaccel);
    myFile.print(",");
    myFile.print(" ay: ");
    myFile.print(yaccel);
    myFile.print(",");
    myFile.print(" az: ");
    myFile.print(zaccel);
    myFile.print(",");
    // deg/s
    myFile.print(" gx: ");
    myFile.print(roll_rate*RAD_TO_DEG);
    myFile.print(",");
    myFile.print(" gy: ");
    myFile.print(pitch_rate*RAD_TO_DEG);
    myFile.print(",");
    myFile.print(" gz: ");
    myFile.print(yaw_rate*RAD_TO_DEG);
    myFile.print(",");
    // uT
    myFile.print(" mx: ");
    myFile.print(xmag);
    myFile.print(",");
    myFile.print(" my: ");
    myFile.print(ymag);
    myFile.print(",");
    myFile.print(" mz: ");
    myFile.print(zmag);
    myFile.print(",");
    // deg
    myFile.print(" phi: ");
    myFile.print(comp_roll_angle*RAD_TO_DEG);
    myFile.print(",");
    myFile.print(" theta: ");
    myFile.print(comp_pitch_angle*RAD_TO_DEG);
    myFile.print(",");
    myFile.print(" psi: ");
    myFile.print(comp_yaw_angle*RAD_TO_DEG);
    myFile.print(",");
    // Reads Temperature (F)
    myFile.print(" Temp: ");
    myFile.print(temp);
    myFile.print(",");
    // Reads Altitude (m)
    myFile.print(" Alt: ");
    myFile.print(currentaltitude);
    myFile.print(",");
    // Reads Pressure (psi)
    myFile.print(" Pres: ");
    myFile.print(pres);
    myFile.print(",");
    // Reads GPS Data
      // if the encode function returns true, it means new GPS data is available
      // you can now access the parsed GPS data using the TinyGPS++ library functions
    myFile.print(" Lat: ");
    myFile.print(gps.location.lat(), 6);
    myFile.print(",");
    myFile.print(" Long: ");
    myFile.print(gps.location.lng(), 6);
    myFile.print(",");
    myFile.print(" # of Sats: ");
    myFile.println(gps.satellites.value());
    myFile.close();
  }
}


void _setColor()
{
    analogWrite(RED_LED_PIN, r);
    analogWrite(WHITE_LED_PIN, w);
    analogWrite(GREEN_LED_PIN, g);
    tone(BUZZER_PIN, buz);
}
void setColor(int red, int green, int white, int buzz)
{
    //Bootup-Light Effect
    while (r != red || g != green || w != white || buz != buzz) {
        if (r < red)
            r += 1;
        if (r > red)
            r -= 1;

        if (g < green)
            g += 1;
        if (g > green)
            g -= 1;

        if (w < white)
            w += 1;
        if (w > white)
            w -= 1;

        if (buz < buzz)
            buz += 10;
        if (buz > buzz)
            buz -= 10;

        _setColor();
        delay(5);
    }

    if (buz == 1800) {
        //buzzer tone
        noTone(BUZZER_PIN);
        delay(500);
        tone(BUZZER_PIN, 1800, 100);
        delay(100);
        noTone(BUZZER_PIN);
        tone(BUZZER_PIN, 1800, 100);
        delay(100);
        noTone(BUZZER_PIN);
    }
}
void startup()
{
    //LED gradient effect
    setColor(255, 0, 0, 0); //red
    setColor(0, 0, 255, 450); //blue
    setColor(255, 0, 255, 1800);
}

void calibration(){
  // Calibration Section
  Serial.println("Calibration Beginning");
  delay(1000);
  Serial.println("Calibrating Accel & Gyro");
  delay(2000);
  imu.calibrateAccel();
  imu.calibrateGyro();

  Serial.println("Calibrating Mag");
  delay(2000);
  imu.calibrateMag();
}

void protocol(){
  state = 0;
  delay(750);
  Serial.println("Flight Computer Ready!");
  delay(500);
}
// put function definitions here:
void flight() {

  Serial.print("t: ");
  Serial.print(currenttime/1000, 3);
  // Complimentary Filter
  
  Serial.print(", ax: ");
  Serial.print(xaccel, 1);
  Serial.print(", ay: ");
  Serial.print(yaccel, 1);
  Serial.print(", az: ");
  Serial.print(zaccel, 1);
  
  Serial.print(",");
  Serial.print(" roll: ");
  Serial.print((comp_roll_angle * RAD_TO_DEG), 3);
  Serial.print(",");
  Serial.print(" pitch: ");
  Serial.print((comp_pitch_angle * RAD_TO_DEG), 3);
  Serial.print(",");
  Serial.print(" yaw: ");
  Serial.print((comp_yaw_angle * RAD_TO_DEG), 3);

  Serial.print(", mx: ");
  Serial.print(xmag, 1);
  Serial.print(", my: ");
  Serial.print(ymag, 1);
  Serial.print(", mz: ");
  Serial.print(zmag, 1);
  /*
  Serial.print(",");
  Serial.print(" roll: ");
  Serial.print((comp_roll_angle_dos * RAD_TO_DEG), 3);
  */
}

// for tomorrow
// Data Logging
// Initilization/Calibration
// Include GPS
