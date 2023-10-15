#include <Arduino.h>
#include <MPU9250.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP085.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

//Sensor Defining
MPU9250 imu(Wire, 0x68);
Adafruit_BMP085 bmp;
TinyGPSPlus gps;

//GPS Varibles
static const int rxPin = 0, txPin= 1; //Pins used for Communication
static const uint32_t GPSBaud = 9600; //Baud Rate
SoftwareSerial ss(rxPin, txPin);      //RX & TX in Serial

//Define Some Variables
float XAccel; float YAccel; float ZAccel;
float XGyro; float YGyro; float ZGyro;
float XMag; float YMag; float ZMag;
float Pres; float Temp;

//Define some Pins
#define pushbutton 7
#define buzzer 28
#define redled 40
#define whiteled 15
//int solenoidPin_1 = 13; int solenoidPin_2 = 14;

//Math Portion
float Roll_Angle; float Pitch_Angle; float Yaw_Angle;
float Roll_Old; float Pitch_Old; // for filtering
float Roll; float Pitch; float Yaw;
float moment_inertia; float angular_moment;
float torque; float mass_flow;
/*
float mass = 10; float radius = 5;
float thrust = 145.6; float isp = 55.51;
float Vtwo; float density = 1.87;
float fire;
*/

// Time Portion
float dt {0.0f};
float millisOld {0.0f};

float currTime {0.0f}; 
float prevTime {0.0f}; 

// File Setup
File myFile;
int brightness;

//Void Functions
void StateMachine();
void displayState(String currState);

void setup() {

  //Serial for baudrate
  Serial.begin(9600);

  //Pin out for States
  pinMode(redled, OUTPUT);
  pinMode(whiteled, OUTPUT);
  pinMode(pushbutton, INPUT);
  pinMode(buzzer, OUTPUT);

  //Starting State
  digitalWrite(buzzer, LOW);
  digitalWrite(redled, LOW);
  digitalWrite(whiteled, LOW);

  

  //Data Logging Initilization
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
    while(1);
  }
  Serial.println("SD card initialized.");
  //Open the file
    myFile = SD.open("F_DATA.txt", FILE_WRITE);
  if (myFile) {
    myFile.println("Time,AccX,AccY,AccZ,Roll,Pitch,Yaw,Roll_Angle,Pitch_Angle,Yaw_Angle");
    //myFile.close();
  } else {
    Serial.println("Error opening data file!");
  }

  //Initializing Sensors
  //IMU
  imu.begin();
  // setting the accelerometer full scale range to +/-8G 
  imu.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  imu.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  // setting DLPF bandwidth to 20 Hz
  imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  imu.setSrd(20);
  //Barometric Pressure Sensor
  if (!bmp.begin()) {
	Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	while (1) {}
  }
  //Starting GPS
  ss.begin(GPSBaud);
      
}

void loop() {
  //TimeStep
  currTime = millis();
  dt = (currTime - prevTime)/1000;

  // Reads the Data
  imu.readSensor();
  //Acceleration (m/s)
  XAccel = imu.getAccelX_mss();
  YAccel = imu.getAccelY_mss();
  ZAccel = imu.getAccelZ_mss();
  //Gyroscope (rot/s)
  XGyro = imu.getGyroX_rads();
  YGyro = imu.getGyroY_rads();
  ZGyro = imu.getGyroZ_rads();
  //Magnometer ()
  XMag = imu.getMagX_uT();
  YMag = imu.getMagY_uT();
  ZMag = imu.getMagZ_uT();

  Pres = bmp.readPressure();
  Temp = bmp.readTemperature();
/*
  // display the data
  //Serial.print(" AX: ");
  Serial.print(XAccel,3);
  Serial.print("\t");
  //Serial.print(" AY: ");
  Serial.print(YAccel,3);
  Serial.print("\t");
  //Serial.print(" AZ: ");
  Serial.print(ZAccel,3);
  Serial.print("\t");
  //Serial.print(" GX: ");
  Serial.print(XGyro,3);
  Serial.print("\t");
  //Serial.print(" GY: ");
  Serial.print(YGyro,3);
  Serial.print("\t");
  //Serial.print(" GZ: ");
  Serial.print(ZGyro,3);
  Serial.print("\t");
  //Serial.print(" MX: ");
  Serial.print(XMag,3);
  Serial.print("\t");
  //Serial.print(" MY: ");
  Serial.print(YMag,3);
  Serial.print("\t");
  //Serial.print(" MZ: ");
  Serial.print(ZMag,3);
  Serial.print("\t");
*/
  //Angle Math
  Roll_Angle = Roll_Angle + ((XGyro*(180/(3.1416))) * dt);
  Pitch_Angle = Pitch_Angle + ((YGyro*(180/(3.1416))) * dt);
  Yaw_Angle = Yaw_Angle + ((ZGyro*(180/(3.1416))) * dt);

  Roll_Old = Roll + ((XGyro*(180/(3.1416))) * dt);
  Pitch_Old = Pitch + ((YGyro*(180/(3.1416))) * dt);

  //Complementary filter, not for Yaw b/c doesn't have one
  Roll = 0.96 * Roll_Old + 0.04 * XAccel;
  Pitch = 0.96 * Pitch_Old + 0.04 * YAccel;

  StateMachine();
  /*
  //Section for Rotation
  //need to be able to tell code, to know what it's original position was
  //to determine how far it moves from that original position
  if (abs(Yaw) > Yaw) {
  }
  */
  prevTime = currTime;
  delay(5);
}

void StateMachine() {
  enum class flightstate : uint8_t {
    IDLE,
    FLIGHT_MODE,
    FOUND,
  };

  static flightstate currState = flightstate::IDLE;
  static bool buttonPressed = false;

  //idle state 1 with both leds off
  switch (currState) {
    case flightstate::IDLE:
      displayState("IDLE MODE");
      digitalWrite(redled, LOW);
      digitalWrite(buzzer, HIGH);
      delay(70);
      digitalWrite(buzzer, LOW);
      delay(70);
      digitalWrite(buzzer, HIGH);
      digitalWrite(whiteled, HIGH);
      delay(100);
      digitalWrite(whiteled, LOW);
      delay(100);
      digitalWrite(whiteled, HIGH);

      if (digitalRead(pushbutton) == 0 && !buttonPressed) {
        buttonPressed = true;
        currState = flightstate::FLIGHT_MODE;
      }
      break;

  //state 2 with led 1 working
    case flightstate::FLIGHT_MODE:
      displayState("FLIGHT_MODE");
      digitalWrite(whiteled, LOW);
      digitalWrite(buzzer, HIGH);
      delay(120);
      digitalWrite(buzzer, LOW);
      delay(120);
      digitalWrite(buzzer, HIGH);
      digitalWrite(redled, HIGH);
      delay(100);
      digitalWrite(redled, LOW);
      delay(100);
      digitalWrite(redled, HIGH);

    Serial.print("Time: ");
    Serial.print(currTime/1000, 3);
    Serial.print("s ");
    Serial.print("\t");
    
    //Print Statements
    Serial.print(" Roll_Angle: ");
    Serial.print(Roll_Angle,3); 
    Serial.print("\t");
    Serial.print(" Pitch_Angle: ");
    Serial.print(Pitch_Angle,3);
    Serial.print("\t");
    Serial.print(" Yaw_Angle: ");
    Serial.print(Yaw_Angle,3); 
    Serial.print("\t");

    Serial.print(" Roll: ");
    Serial.print(Roll,3); 
    Serial.print("\t");
    Serial.print(" Pitch: ");
    Serial.print(Pitch,3);
    Serial.print("\t");
    Serial.print(" Yaw: ");
    Serial.print(ZGyro,3); 
    Serial.print("\t");

    //Reads Temperature
    Serial.print(" T: ");
    Serial.print(Temp);
    Serial.print("\t");

    //Reads Pressure
    Serial.print(" P: ");
    Serial.print(Pres);

    //brightness = EEPROM.read(0);
    //Serial.print(brightness);
    //delay(500);
    
    Serial.println();
/*
  //Reads GPS Data
    while (ss.available() > 0){
      // get the byte data from the GPS
      byte gpsData = ss.read();
      // parse the byte using the encode function
      if (gps.encode(gpsData)) {
        // if the encode function returns true, it means new GPS data is available
        // you can now access the parsed GPS data using the TinyGPS++ library functions
        Serial.print(" Lat: ");
        Serial.print(gps.location.lat(), 6);
        Serial.print(" Long: ");
        Serial.print(gps.location.lng(), 6);
        Serial.print(" # of Sats= "); 
        Serial.println(gps.satellites.value()); 
      }
    }
*/
      //Data Logging Part
    myFile = SD.open("F_DATA.txt", FILE_WRITE);
    if (myFile) {
      myFile.print(currTime/1000, 3);
      myFile.print(",");
      myFile.print(XAccel);
      myFile.print(",");
      myFile.print(YAccel);
      myFile.print(",");
      myFile.print(ZAccel);
      myFile.print(",");
      myFile.print(Roll);
      myFile.print(",");
      myFile.print(Pitch);
      myFile.print(",");
      myFile.print(ZGyro);
      myFile.print(",");
      myFile.print(Roll_Angle);
      myFile.print(",");
      myFile.print(Pitch_Angle);
      myFile.print(",");
      myFile.print(Yaw_Angle);
            //Reads Temperature
      myFile.print(" T: ");
      myFile.print(Temp);

      //Reads Pressure
      myFile.print(" P: ");
      myFile.print(Pres);
      myFile.println();
      myFile.close();

/*
    //Reads GPS Data
      while (ss.available() > 0){
        // get the byte data from the GPS
        byte gpsData = ss.read();
        // parse the byte using the encode function
        if (gps.encode(gpsData)) {
          // if the encode function returns true, it means new GPS data is available
          // you can now access the parsed GPS data using the TinyGPS++ library functions
          myFile.print(" Lat: ");
          myFile.print(gps.location.lat(), 6);
          myFile.print(" Long: ");
          myFile.print(gps.location.lng(), 6);
          myFile.print(" # of Sats= "); 
          myFile.println(gps.satellites.value()); 
        }
      }
*/

    } else {
      Serial.println("Error opening data file!");
    }

      if (digitalRead(pushbutton) == 0 && !buttonPressed) {
        buttonPressed = true;
        currState = flightstate::FOUND;
      }
      break;

  //state 3 with led 2 working
    case flightstate::FOUND:
      displayState("FOUND");
      digitalWrite(redled, LOW);
      digitalWrite(whiteled, LOW);

      if (digitalRead(pushbutton) == 0 && !buttonPressed) {
        buttonPressed = true;
        currState = flightstate::IDLE;
      }
      break;

    default:

      Serial.println("'Default' Switch Case reached - Error");
  }

  if (digitalRead(pushbutton) == 1) {
    buttonPressed = false;
  }
}

//helper to print state the system is at
void displayState(String currState) {
    static String prevState = "";

    if (currState != prevState) {
        Serial.println(currState);
        prevState = currState;
    }
}
