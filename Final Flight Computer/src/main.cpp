#include <Arduino.h>
#include <MPU9250.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP085.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

// Sensor Deifinitions
MPU9250 imu(Wire, 0x68);
Adafruit_BMP085 bmp;
TinyGPSPlus gps;

int imu_status;
double gps_status;
int state;

// ------------ IMU Variables -------------- //
// Accelerometer
double xaccel = 0, yaccel = 0, zaccel = 0;
// Gyroscope
double roll_rate = 0, pitch_rate = 0, yaw_rate = 0;
double roll_angle = 0, pitch_angle = 0, yaw_angle = 0;
// Magnometer
double xmag = 0, ymag = 0, zmag = 0;

float i_temp;

// BMP Variables
float Pres;
float b_temp;

// GPS Variables
static const int rxPin = 0;
static const int txPin = 1;
static const uint32_t GPSBaud = 9600;
SoftwareSerial ss(rxPin, txPin);

// Hardware Stuff
#define pushbutton 25
#define buzzer 26
#define redled 37
#define whiteled 38
#define greenled 35

#define pyro1 56
#define pyro2 57

//Continuity Data/LEDs
int cnt1 = 9;
int cnt2 = 10;
int p1 = 11; //led to indicate continuity
int p2 = 12; //led to indicate continuity
int read1, read2;

int r = 0;
int w = 0;
int g = 0;
int buz = 0;

// Math Portion
// Altitude
float initialaltitude;
float PressureAtSea = 101388.48; // Pa // chage for pressure here
float maxaltitude = 0;
double temp;

// Filters
int lpfGyr = 0.7;
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

// Storage SetpUp
File myFile;

void print()
{
  /////////  Printing Portion  //////////

  Serial.print("time: ");
  Serial.print(currenttime / 1000, 3);
  Serial.print("\t");

  // Complimentary Filter
  Serial.print(" ax: ");
  Serial.print(xaccel, 6);
  Serial.print("\t");
  Serial.print(" ay: ");
  Serial.print(yaccel, 6);
  Serial.print("\t");
  Serial.print(" az: ");
  Serial.print(zaccel, 6);
  Serial.print("\t");
  Serial.print(" roll: ");
  Serial.print((comp_roll_angle * RAD_TO_DEG) + 180, 6);
  Serial.print("\t");
  Serial.print(" pitch: ");
  Serial.print((comp_pitch_angle * RAD_TO_DEG) + 180, 6);
  Serial.print("\t");
  Serial.print(" yaw: ");
  Serial.print((comp_yaw_angle * RAD_TO_DEG) + 180, 6);
  Serial.print("\t");
  Serial.print(" Roll_Rate: ");
  Serial.print(roll_rate * RAD_TO_DEG, 6);
  Serial.print("\t");
  Serial.print(" Pitch_Rate: ");
  Serial.print(pitch_rate * RAD_TO_DEG, 6);
  Serial.print("\t");
  Serial.print(" Yaw_Rate: ");
  Serial.print(yaw_rate * RAD_TO_DEG, 6);
  Serial.print("\t");
  Serial.print(" imu Temp: ");
  Serial.print(i_temp, 6);
  Serial.print("\t");
  Serial.print(" baro Temp: ");
  Serial.print(b_temp, 6);
  Serial.println();
}
void displayInfo();
void _setColor()
{
    analogWrite(redled, r);
    analogWrite(whiteled, w);
    analogWrite(greenled, g);
    tone(buzzer, buz);
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
        noTone(buzzer);
        delay(500);
        tone(buzzer, 1800, 100);
        delay(100);
        noTone(buzzer);
        tone(buzzer, 1800, 100);
        delay(100);
        noTone(buzzer);
    }
}
void startup()
{
    //LED gradient effect
    setColor(255, 0, 0, 0); //red
    setColor(0, 0, 255, 450); //blue
    setColor(255, 0, 255, 1800);
}
void protocol()
{
  state = 0;
  delay(750);
  Serial.println("Flight Computer Ready!");
  //Accel, Gyro, and Mag Initialization
  /*
  xaccel = imu.getAccelX_mss();
  zaccel = imu.getAccelZ_mss();
  roll_rate = imu.getGyroX_rads();
  pitch_rate = imu.getGyroY_rads();
  yaw_rate = imu.getGyroZ_rads();
  xmag = imu.getMagX_uT();
  ymag = imu.getMagY_uT();
  zmag = imu.getMagZ_uT();

  int statusA;
  int statusG;
  int statusM;

  statusA = (xaccel + yaccel + zaccel) / 3;
  statusG = (roll_rate + pitch_rate + yaw_rate) / 3;
  statusM = (xmag + ymag + zmag) / 3;

    if (statusA = 0)
  {
    Serial.println("Accel Initialization Error");
    while (1)
    {
    }
  }
  if (statusG = 0)
  {
    Serial.println("Gyro Initialization Error");
    while (1)
    {
    }
  }
  if (statusM = 0)
  {
    Serial.println("Mag Initialization Error");
    while (1)
    {
    }
  }
  */
 /*
  //IMU Initilization
  if (imu_status < 0)
  {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(imu_status);
    while (1)
    {
    }
  }
  // BMP Inilization
  if (!bmp.begin())
  {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1)
    {
    }
  }
 */ 
  delay(500);
  digitalWrite(redled, HIGH);
  digitalWrite(whiteled, HIGH);
}
void deploy()
{
  int potVal = analogRead(A0); // potentiometer values for time delay in deployment
  int TimeDelay = ((-(potVal) + 1023) / 102.3) * 1000;
  digitalWrite(redled, HIGH);
  digitalWrite(greenled, HIGH);
  Serial.print("\n");
  delay(TimeDelay);
  state++;
  digitalWrite(pyro1, HIGH);
  digitalWrite(pyro2, HIGH);
  digitalWrite(p1, HIGH);
  digitalWrite(p2, HIGH);
  tone(buzzer, 1800);
  delay(5000);
  state++;
  while (state == 3)
  {
    digitalWrite(pyro1, LOW);
    digitalWrite(pyro2, LOW);
    digitalWrite(redled, HIGH);
    digitalWrite(whiteled, HIGH);
    digitalWrite(greenled, HIGH);
    tone(buzzer, 2000, 500);
    delay(500);
    digitalWrite(redled, HIGH);
    digitalWrite(whiteled, LOW);
    digitalWrite(greenled, LOW);
    delay(100);
  }
}
void flight()
{
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

  roll_angle += ((roll_rate * RAD_TO_DEG) * elapsedtime);
  pitch_angle += ((pitch_rate * RAD_TO_DEG) * elapsedtime);
  yaw_angle += ((yaw_rate * RAD_TO_DEG) * elapsedtime);

  // Complimentary Filter w/ mag
  comp_roll_angle = alpha * (roll_angle + roll_rate * elapsedtime) + (1 - alpha) * mag_roll_angle;
  comp_pitch_angle = alpha * (pitch_angle + pitch_rate * elapsedtime) + (1 - alpha) * mag_pitch_angle;
  comp_yaw_angle = alpha * (yaw_angle + yaw_rate * elapsedtime) + (1 - alpha) * mag_yaw_angle;

  // BMP
  b_temp = bmp.readTemperature();
  print();
  while (ss.available() > 0)
  {
    // get the byte data from the GPS
    byte gpsData = ss.read();
    // parse the byte using the encode function
    if (gps.encode(gpsData))
    {
      // if the encode function returns true, it means new GPS data is available
      // you can now access the parsed GPS data using the TinyGPS++ library functions
      Serial.print(" Lat: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(" Long: ");
      Serial.print(gps.location.lng(), 6);
      Serial.print(" # of Sats: ");
      Serial.println(gps.satellites.value());
    }
  }

  float currentaltitude = bmp.readAltitude(PressureAtSea) - initialaltitude;
  if (currentaltitude > maxaltitude)
  {
    maxaltitude = currentaltitude;
  }

  if (maxaltitude > currentaltitude + 0.75)
  {
    deploy();
  }

  previoustime = currenttime;
}
void launchdetect()
{
  float speed = yaccel;

  if (state == 0 && speed > 2)
  {
    // Default set to 60m/s^2
    state++;
  }

  if (state == 1)
  {
    flight();
  }
}

void setup()
{
  Wire.begin();
  // Serial for baudrate
  Serial.begin(115200);

  // Pin out for States
  pinMode(redled, OUTPUT);
  pinMode(whiteled, OUTPUT);
  pinMode(greenled, OUTPUT);
  pinMode(pushbutton, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(pyro1, OUTPUT);
  pinMode(pyro2, OUTPUT);
  pinMode(cnt1, OUTPUT);
  pinMode(cnt2, OUTPUT);
  pinMode(p1, OUTPUT);
  pinMode(p2, OUTPUT);
  /*
    // Data Logging Initilization
    if (!SD.begin(BUILTIN_SDCARD))
    {
      Serial.println("SD card initialization failed!");
      while (1) {}
    }
    Serial.println("SD card initialized.");
    // Open the file
    myFile = SD.open("Test_DATA.txt", FILE_WRITE);
    if (myFile)
    {
      myFile.println(" ---------- Flight Data -----------");
      myFile.close();
    }
    else
    {
      Serial.println("Error opening data file!");
    }
  */
  delay(2000);
  
  imu_status = imu.begin();
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
  // Starting GPS
  ss.begin(GPSBaud);

  startup();
  protocol();
}

void loop()
{
  imu.readSensor();
  xaccel = lpfAcc * xaccel + (1 - lpfAcc) * imu.getAccelX_mss();
  yaccel = lpfAcc * yaccel + (1 - lpfAcc) * imu.getAccelY_mss();
  Serial.println(yaccel);
  currenttime = millis();
  elapsedtime = (currenttime - previoustime) / 1000;
  launchdetect();
  int read1 = analogRead(A1);
  int read2 = analogRead(A2);
  if (read1 >= 200)
  {
    digitalWrite(cnt1, HIGH);
  }
  else
  {
    digitalWrite(cnt1, LOW);
  }

  if (read2 >= 200)
  {
    digitalWrite(cnt2, HIGH);
  }
  else
  {
    digitalWrite(cnt2, LOW);
  }

  Serial.flush();
}


/*
void calibration(){
  
}
*/
/*
void displayInf()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}
*/
/*
void datalogging()
{
  // Data Logging Part
  myFile = SD.open("Test_DATA.txt", FILE_WRITE);
  if (myFile)
  {
    myFile.print(" time: ");
    myFile.print(currenttime / 1000, 3);
    myFile.print(",");
    myFile.print(" ax: ");
    myFile.print(xaccel);
    myFile.print(",");
    myFile.print(" ay: ");
    myFile.print(yaccel);
    myFile.print(" az: ");
    myFile.print(",");
    myFile.print(zaccel);
    myFile.print(" gx: ");
    myFile.print(",");
    myFile.print(roll_rate);
    myFile.print(" gy: ");
    myFile.print(",");
    myFile.print(pitch_rate);
    myFile.print(" gz: ");
    myFile.print(",");
    myFile.print(yaw_rate);
    myFile.print(" phi: ");
    myFile.print(",");
    myFile.print(comp_roll_angle);
    myFile.print(" theta: ");
    myFile.print(",");
    myFile.print(comp_pitch_angle);
    myFile.print(" psi: ");
    myFile.print(",");
    myFile.print(comp_yaw_angle);

    // Reads Temperature
    myFile.print(" Temp: ");
    myFile.print(temp);

    // Reads Pressure
    myFile.print(" Pres: ");
    myFile.print(Pres);
    myFile.println();
    // Reads GPS Data
    while (ss.available() > 0)
    {
      // get the byte data from the GPS
      byte gpsData = ss.read();
      // parse the byte using the encode function
      if (gps.encode(gpsData))
      {
        // if the encode function returns true, it means new GPS data is available
        // you can now access the parsed GPS data using the TinyGPS++ library functions
        myFile.print(" Lat: ");
        myFile.print(gps.location.lat(), 6);
        myFile.print(" Long: ");
        myFile.print(gps.location.lng(), 6);
        myFile.print(" # of Sats: ");
        myFile.println(gps.satellites.value());
      }
    }
    myFile.close();
  }
}
*/
/*
void continuity()
{
  // Activate the igniter pin to test continuity
  digitalWrite(pyro1, HIGH);
  delay(100); // Provide a short pulse

  // Check continuity using a digitalRead
  int continuity = digitalRead(pyro1);

  // Display the result or take action based on continuity
  if (continuity == HIGH)
  {
    Serial.println("Continuity detected - Igniter circuit is complete.");
    // Perform actions related to continuity
  }
  else
  {
    Serial.println("No continuity - Igniter circuit is open.");
    // Handle lack of continuity (circuit open)
  }

  delay(5000); // Delay before next test

  digitalWrite(pyro1, HIGH);
  delay(100); // Provide a short pulse

  // Check continuity using a digitalRead
  int continuity = digitalRead(pyro2);

  // Display the result or take action based on continuity
  if (continuity == HIGH)
  {
    Serial.println("Continuity detected - Igniter circuit is complete.");
    // Perform actions related to continuity
  }
  else
  {
    Serial.println("No continuity - Igniter circuit is open.");
    // Handle lack of continuity (circuit open)
  }

  delay(5000); // Delay before next test
}
*/