#include <Arduino.h>
#include <MPU9250.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP085.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

MPU9250 imu(Wire, 0x68);
Adafruit_BMP085 bmp;
TinyGPSPlus gps;

int state;
bool hasLaunched = false;
bool haspressed = false;

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
#define BUTTON_PIN 25
#define BUZZER_PIN 26
#define RED_LED_PIN 37
#define WHITE_LED_PIN 38
#define GREEN_LED_PIN 35
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
double alpha = 0.93f;
// Complimentary filter variables
double accel_roll_angle = 0, accel_pitch_angle = 0, accel_yaw_angle;
double mag_roll_angle = 0, mag_pitch_angle = 0, mag_yaw_angle = 0;
double comp_roll_angle = 0, comp_pitch_angle = 0, comp_yaw_angle = 0;
// Bias
double bias_roll_rate, bias_pitch_rate, bias_yaw_rate;

// Time
float elapsedtime{0.0f};
float currenttime{0.0f};
float previoustime{0.0f};

File myFile;

// put function declarations here:
void flight();
void launchdetect();
void deploy();
void protocol();
void startup();
void datalogging();

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
  myFile = SD.open("Jan_first_Test_Data.txt", FILE_WRITE);
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

  accel_roll_angle = atan2(-xaccel, sqrt(pow(yaccel, 2) + pow(zaccel, 2))) * RAD_TO_DEG;
  accel_pitch_angle = atan2(yaccel, sqrt(pow(xaccel, 2) + pow(zaccel, 2))) * RAD_TO_DEG;
  accel_yaw_angle = atan2((sqrt(pow(xaccel, 2) + pow(yaccel, 2))), zaccel) * RAD_TO_DEG;
  mag_roll_angle = atan2(ymag, zmag);
  mag_pitch_angle = atan2(xmag, zmag);
  mag_yaw_angle = atan2(ymag, xmag);

  roll_angle += (roll_rate * elapsedtime);
  pitch_angle += (pitch_rate * elapsedtime);
  yaw_angle += (yaw_rate  * elapsedtime);

  // Complimentary Filter w/ mag
  comp_roll_angle = alpha * (roll_angle + roll_rate * elapsedtime) + (1 - alpha) * mag_roll_angle;
  comp_pitch_angle = alpha * (pitch_angle + pitch_rate * elapsedtime) + (1 - alpha) * mag_pitch_angle;
  comp_yaw_angle = alpha * (yaw_angle + yaw_rate * elapsedtime) + (1 - alpha) * mag_yaw_angle;

  //complimentary filter for position with x, and xdot from accel
  //then use gps similar to mag above, but make sure to convert the position unit stuff
  //so essentially it'll be just converting x and y position to lat and long

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
    datalogging();
  }
  
  previoustime = currenttime;
}

void datalogging(){
  // Data Logging Part
  myFile = SD.open("Jan_first_Test_Data.txt", FILE_WRITE);
  if (myFile)
  {
    myFile.print(" time: ");
    myFile.print(currenttime / 1000, 3);
    myFile.print(",");
    myFile.print(" ax: ");
    myFile.print(xaccel);
    myFile.print(",");
    myFile.print(" ay: ");
    myFile.print(",");
    myFile.print(yaccel);
    myFile.print(" az: ");
    myFile.print(zaccel);
    myFile.print(",");
    myFile.print(" gx: ");
    myFile.print(roll_rate* RAD_TO_DEG);
    myFile.print(",");
    myFile.print(" gy: ");
    myFile.print(pitch_rate* RAD_TO_DEG);
    myFile.print(",");
    myFile.print(" gz: ");
    myFile.print(yaw_rate* RAD_TO_DEG);
    myFile.print(",");
    myFile.print(" phi: ");
    myFile.print(comp_roll_angle* RAD_TO_DEG);
    myFile.print(",");
    myFile.print(" theta: ");
    myFile.print(comp_pitch_angle* RAD_TO_DEG);
    myFile.print(",");
    myFile.print(" psi: ");
    myFile.print(comp_yaw_angle* RAD_TO_DEG);
    myFile.print(",");

    // Reads Temperature
    myFile.print(" Temp: ");
    myFile.print(temp);
    myFile.print(",");
    
    myFile.print(" Alt: ");
    myFile.print(currentaltitude);
    myFile.print(",");

    // Reads Pressure
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

void protocol(){
  state = 0;
  delay(750);
  Serial.println("Flight Computer Ready!");
  delay(500);
  digitalWrite(GREEN_LED_PIN, HIGH);
}
// put function definitions here:
void flight() {

  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(WHITE_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);

  Serial.print("t: ");
  Serial.print(currenttime / 1000, 3);
  Serial.print("\t");

  // Complimentary Filter
  Serial.print(" ax: ");
  Serial.print(xaccel, 1);
  Serial.print("\t");
  Serial.print(" ay: ");
  Serial.print(yaccel, 1);
  Serial.print("\t");
  Serial.print(" az: ");
  Serial.print(zaccel, 1);
  Serial.print("\t");
  Serial.print(" roll: ");
  Serial.print((comp_roll_angle * RAD_TO_DEG), 1);
  Serial.print("\t");
  Serial.print(" pitch: ");
  Serial.print((comp_pitch_angle * RAD_TO_DEG), 1);
  Serial.print("\t");
  Serial.print(" yaw: ");
  Serial.print((comp_yaw_angle * RAD_TO_DEG), 1);
  Serial.print("\t");
  Serial.print(" gx: ");
  Serial.print(roll_rate * RAD_TO_DEG, 1);
  Serial.print("\t");
  Serial.print(" gy: ");
  Serial.print(pitch_rate * RAD_TO_DEG, 1);
  Serial.print("\t");
  Serial.print(" gz: ");
  Serial.print(yaw_rate * RAD_TO_DEG, 1);
  //Serial.print("\t");
  //Serial.print(" T: ");
  //Serial.print(temp, 2);
  Serial.print("\t");
  Serial.print(" Alt: ");
  Serial.print(currentaltitude, 1);
  //Serial.print("\t");
  //Serial.print(" Spres: ");
  //Serial.print(sealevelpres/6895, 4);
  Serial.print("\t");
  Serial.print(" pres: ");
  Serial.print(pres, 1);
  Serial.print(" Lat: ");
  Serial.print(gps.location.lat(), 2);
  Serial.print(" Long: ");
  Serial.print(gps.location.lng(), 2);
  Serial.print(" # of Sats: ");
  Serial.println(gps.satellites.value());

  float currentaltitude = bmp.readAltitude(sealevelpres) - initialaltitude;
  if (currentaltitude > maxaltitude)
  {
    maxaltitude = currentaltitude;
  }

  if (maxaltitude > currentaltitude + 15)
  {
    Serial.println("---------");
    Serial.println("---------");
    Serial.println("parachute!");
    Serial.println("---------");
    Serial.println("---------");
    deploy();
  }
}


void launchdetect()
{
  float speed = yaccel;

  if (state == 0 && speed > 13)
  {
    // Default set to 60m/s^2
    state++;
  }

  if (state == 1 && !hasLaunched)
  {
    Serial.println("---------");
    Serial.println("---------");
    Serial.println("launched!");
    Serial.println("---------");
    Serial.println("---------");
    hasLaunched = true;
    //flight();
  }
}

void deploy()
{
  delay(5000);
  state++;
  while (state == 2)
  {
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(WHITE_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, HIGH);
    tone(BUZZER_PIN, 2000, 500);
    delay(100);
  }
}

// for tomorrow
// Data Logging
// Initilization/Calibration
// Include GPS

