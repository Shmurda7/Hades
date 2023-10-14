#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>

//naming the sensors
Adafruit_MPU6050 mpu; //MPU6050 
Adafruit_BMP085 bmp;  //BMP180
TinyGPSPlus gps;      //GPS NEO-6M


//Define the Variables for the Sensors
  //Time
float dt {0.0f};
float millisOld {0.0f};

float currTime {0.0f};
float prevTime {0.0f}; 
  //IMU
float XAccel; float YAccel; float ZAccel;
float XGyro; float YGyro; float ZGyro;
float Temp;
  //Angular Position
float Roll_Old; float Pitch_Old; // for filtering
float Roll; float Pitch; float Yaw;
  //BARO
float Pres;
  //GPS
static const int rxPin = 0, txPin= 1; //Pins used for Communication
static const uint32_t GPSBaud = 9600; //Baud Rate
SoftwareSerial ss(rxPin, txPin);      //IDK


void setup() {
  //FOR E-MATCH
  pinMode(13, OUTPUT);
  //IDK
  Serial.begin(9600);
  //Starting and Setting Ranges for IMU
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  //Starting BARO
  bmp.begin();
  //Starting GPS
  ss.begin(GPSBaud);


  //static boolean b =true;
  //Serial.print(b ?);
}

void loop() {
  /*
  //was used when trying to turn on E-match
  delay(500);
  digitalWrite(13, HIGH);
  Serial.println(" On");   
  delay(1000);              
  digitalWrite(13, LOW);  
  Serial.println(" Off"); 
  */

//Time Step
currTime = millis();
dt = (currTime - prevTime)/1000;
Serial.print("Time: ");
Serial.print(currTime/1000);
Serial.print("s ");

//Get's Data from Sensors
 sensors_event_t a, g, temp, p; //sensor call change in something an event
//Grab's that Data 
mpu.getEvent(&a, &g, &temp);
bmp.readPressure();

//Collects Accelerometer Data
XAccel = a.acceleration.x;
YAccel = a.acceleration.y;
ZAccel = a.acceleration.z;
Serial.print(" AX: ");
Serial.print(XAccel);
Serial.print(" AY: ");
Serial.print(YAccel);
Serial.print(" AZ: ");
Serial.print(ZAccel);

//Collects Gyroscope Data
XGyro = g.gyro.x;
YGyro = g.gyro.y;
ZGyro = g.gyro.z;
Serial.print(" GX: ");
Serial.print(XGyro);
Serial.print(" GY: ");
Serial.print(YGyro);
Serial.print(" GZ: ");
Serial.print(ZGyro);

//Calculates Angular Position
  //Angluar Position (originally in rad/s)
Roll_Old = Roll + ((XGyro*(180/(3.1416))) * dt);
Pitch_Old = Pitch + ((YGyro*(180/(3.1416))) * dt);
Yaw = Yaw + ((ZGyro*(180/(3.1416))) * dt);
  //Complementary filter, not for Yaw b/c doesn't have one
Roll = 0.96 * Roll_Old + 0.04 * XAccel;
Pitch = 0.96 * Pitch_Old + 0.04 * YAccel;
Serial.print(" Roll: ");
Serial.print(Roll); 
Serial.print(" Pitch: ");
Serial.print(Pitch); 
Serial.print(" Yaw: ");
Serial.print(Yaw); 


//Reads Temperature
Temp = temp.temperature;
Serial.print(" T: ");
Serial.print(Temp);

//Reads Pressure
Pres = p.pressure;
Serial.print(" P: ");
Serial.print(Pres);

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
  Serial.println();
  prevTime = currTime;
}

//test with more data collected from the gps
/*void loop(){
  // This sketch displays information every time a new sentence is correctly encoded.
    while (ss.available() > 0){
    // get the byte data from the GPS
    byte gpsData = ss.read();
    // parse the byte using the encode function
    if (gps.encode(gpsData)) {
      // if the encode function returns true, it means new GPS data is available
      // you can now access the parsed GPS data using the TinyGPS++ library functions
      Serial.print("Latitude: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print("Longitude: ");
      Serial.print(gps.location.lng(), 6);
       
      // Raw latitude in whole degrees
      Serial.print("Raw latitude = "); 
      Serial.print(gps.location.rawLat().negative ? "-" : "+");
      Serial.println(gps.location.rawLat().deg); 
      // ... and billionths (u16/u32)
      Serial.println(gps.location.rawLat().billionths);
      
      // Raw longitude in whole degrees
      Serial.print("Raw longitude = "); 
      Serial.print(gps.location.rawLng().negative ? "-" : "+");
      Serial.println(gps.location.rawLng().deg); 
      // ... and billionths (u16/u32)
      Serial.println(gps.location.rawLng().billionths);

      // Raw date in DDMMYY format (u32)
      Serial.print("Raw date DDMMYY = ");
      Serial.println(gps.date.value()); 

      // Year (2000+) (u16)
      Serial.print("Year = "); 
      Serial.println(gps.date.year()); 
      // Month (1-12) (u8)
      Serial.print("Month = "); 
      Serial.println(gps.date.month()); 
      // Day (1-31) (u8)
      Serial.print("Day = "); 
      Serial.println(gps.date.day()); 

      // Raw time in HHMMSSCC format (u32)
      Serial.print("Raw time in HHMMSSCC = "); 
      Serial.println(gps.time.value()); 

      // Hour (0-23) (u8)
      Serial.print("Hour = "); 
      Serial.println(gps.time.hour()); 
      // Minute (0-59) (u8)
      Serial.print("Minute = "); 
      Serial.println(gps.time.minute()); 
      // Second (0-59) (u8)
      Serial.print("Second = "); 
      Serial.println(gps.time.second()); 
      // 100ths of a second (0-99) (u8)
      Serial.print("Centisecond = "); 
      Serial.println(gps.time.centisecond()); 

      // Raw speed in 100ths of a knot (i32)
      Serial.print("Raw speed in 100ths/knot = ");
      Serial.println(gps.speed.value()); 
      // Speed in knots (double)
      Serial.print("Speed in knots/h = ");
      Serial.println(gps.speed.knots()); 
      // Speed in miles per hour (double)
      Serial.print("Speed in miles/h = ");
      Serial.println(gps.speed.mph()); 
      // Speed in meters per second (double)
      Serial.print("Speed in m/s = ");
      Serial.println(gps.speed.mps()); 
      // Speed in kilometers per hour (double)
      Serial.print("Speed in km/h = "); 
      Serial.println(gps.speed.kmph()); 

      // Raw course in 100ths of a degree (i32)
      Serial.print("Raw course in degrees = "); 
      Serial.println(gps.course.value()); 
      // Course in degrees (double)
      Serial.print("Course in degrees = "); 
      Serial.println(gps.course.deg()); 

      // Raw altitude in centimeters (i32)
      Serial.print("Raw altitude in centimeters = "); 
      Serial.println(gps.altitude.value()); 
      // Altitude in meters (double)
      Serial.print("Altitude in meters = "); 
      Serial.println(gps.altitude.meters()); 
      // Altitude in miles (double)
      Serial.print("Altitude in miles = "); 
      Serial.println(gps.altitude.miles()); 
      // Altitude in kilometers (double)
      Serial.print("Altitude in kilometers = "); 
      Serial.println(gps.altitude.kilometers()); 
      // Altitude in feet (double)
      Serial.print("Altitude in feet = "); 
      Serial.println(gps.altitude.feet()); 

      // Number of satellites in use (u32)
      Serial.print("Number os satellites in use = "); 
      Serial.println(gps.satellites.value()); 

      // Horizontal Dim. of Precision (100ths-i32)
      Serial.print("HDOP = "); 
      Serial.println(gps.hdop.value());
      delay(1000); 
    }
  }
}
*/

