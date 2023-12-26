#include <WiFi.h>
#include "ESPAsyncWebServer.h"
#include <JY901.h>
#include "Adafruit_VL53L0X.h"

// ================================================================
// ===                      General Variables                   ===
// ================================================================
bool autoOn = false;
bool climbing = false;

// ================================================================
// ===                      Lidar Variables                    ===
// ================================================================
VL53L0X_RangingMeasurementData_t measure;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// ================================================================
// ===                      Motor Variables                   ===
// ================================================================
// Motor 1
int motor1Pin1 = 27; 
int motor1Pin2 = 26; 

int motor2Pin1 = 33; 
int motor2Pin2 = 32; 

int cruiseDutyCycle = 180;
int climbDutyCycle = 255;

// ================================================================
// ===                      WiFi Variables                      ===
// ================================================================
// Change the ssid and password to something else 
const char* ssid = "ESP32-Access-Point";
const char* password = "123456789"; 

AsyncWebServer server(80);

// ================================================================
// ===                      WT901 Variables                     ===
// ================================================================
//for uart comm
byte open_com[] = {0xFF, 0xF0, 0xF0, 0xF0, 0xF0};// Special unlock/enable thing (not documented anywhere!)
byte unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
byte save[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
byte change_200Hz[] = {0xFF, 0xAA, 0x03, 0x0B, 0x00}; // 200 hz 
byte calib_gy_and_acc[] = {0xFF, 0xAA, 0x01, 0x01, 0x00};
byte calib_mag[] = {0xFF, 0xAA, 0x01, 0x02, 0x00};
byte calib_z_0[] = {0xFF, 0xAA, 0x01, 0x03, 0x00};
byte exit_calib[] = {0xFF, 0xAA, 0x01, 0x00, 0x00};
byte set_vert[] = {0xFF, 0xAA, 0x23, 0x01, 0x00};
byte set_horiz[] = {0xFF, 0xAA, 0x23, 0x00, 0x00};
byte set_9ax[] = {0xFF, 0xAA, 0x24, 0x00, 0x00};
byte set_ang_only_out[] = {0xFF, 0xAA, 0x02, 0x08, 0x00};
byte set_baud[] = {0xFF, 0xAA, 0x04, 0x07, 0x00};

// generic variables for angles 
double angle_z = 0.0; double angle_x = 0.0; double angle_y = 0.0; 
double offsetx = 0.6317; double offsety = -0.1154; double offsetz = -151.2048;
unsigned long prev901time = 0;
unsigned long dt901 = 0;

void setup() {
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);

  //Calls setup functions for respective components
  Serial.begin(115200);
  setupWifi();
  setup901();

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
}

void loop() {
  //Move autonomously if flag set (done in wifi endpoint)
  if(autoOn){
    moveAuto();
  }
}

// ================================================================
// ===                      General Functions                   ===
// ================================================================
void moveAuto(){
  Serial.println("Moving autonomously...");
  //Check if at stairs
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
  //Turn on full climbing motor power if under range
    if(measure.RangeMilliMeter < 10){
      climbing = true;
        //Move forward until Lidar below certain range
      read_wt901();
      angle_x = (((volatile double)JY901.stcAngle.Angle[0] / 32768 * 180));
      angle_x = -1 * (angle_x - offsetx); // Use X as tilt measurement
      Serial.print("Angle: "); Serial.println(angle_x);

      if(angle_x > 15){
        moveForward(climbDutyCycle);
      } else {
        autoOn = false;
        stopMotor();
      }
    }
  } else {
    moveForward(cruiseDutyCycle);
  }

}

// ================================================================
// ===                      Motor Functions                     ===
// ================================================================
void moveForward(int dutyCycle){
  analogWrite(motor1Pin1, 0);
  analogWrite(motor1Pin2, dutyCycle);

  analogWrite(motor2Pin1, dutyCycle);
  analogWrite(motor2Pin2, 0);
}

void moveBackward(int dutyCycle){
  analogWrite(motor1Pin1, dutyCycle);
  analogWrite(motor1Pin2, 0);

  analogWrite(motor2Pin1, 0);
  analogWrite(motor2Pin2, dutyCycle);
}

void turnRight(int dutyCycle){
  analogWrite(motor1Pin1, dutyCycle);
  analogWrite(motor1Pin2, 0);

  analogWrite(motor2Pin1, dutyCycle);
  analogWrite(motor2Pin2, 0);
}

void turnLeft(int dutyCycle){
  analogWrite(motor1Pin1, 0);
  analogWrite(motor1Pin2, dutyCycle);

  analogWrite(motor2Pin1, 0);
  analogWrite(motor2Pin2, dutyCycle);
}

void stopMotor(){
  analogWrite(motor1Pin1, 0);
  analogWrite(motor1Pin2, 0);

  analogWrite(motor2Pin1, 0);
  analogWrite(motor2Pin2, 0);
}

// ================================================================
// ===                      WT901 Functions                     ===
// ================================================================
void setup901(){
  Serial1.begin(9600, SERIAL_8N1, 17, 16); delay(1000); // pin 16 goes to Rx on chip and 17 to Tx on chip, and connect 3.3 v power and ground 
  set_baud_rate(); delay(1000);
  Serial1.begin(230400, SERIAL_8N1, 17, 16); delay(1000); // pin 16 goes to Rx on chip and 17 to Tx on chip, and connect 3.3 v power and ground 

  set_200_hz(); // calls a function which sets the data rate to 200 Hz
  set_9axis(); // sets the angle  omputation to use accelerometer, gyroscope and magnetometers (9 -axis)

  set_horiz_orien();

  calib_gyro_and_accel(); // calibrates the sensor, leave stationary while this is done 
  exit_calibration(); // finishes and saves calibration
}

// function to read the sensor
void read_wt901(){
  while (Serial1.available()) 
  {
    JY901.CopeSerialData(Serial1.read()); //Call JY901 data cope function
  }
}

// begins the calibrarion functions called in setup 
void set_200_hz()
{
  // Unlock configuration
  Serial1.write(unlock, 5);
  // Serial.println(); 
  delay(1000);
  // Set to 200 HZ
  Serial1.write(change_200Hz, 5);
  delay(1000); // ensure set
  // Save configuration
  Serial1.write(save, 5);
  delay(1000);
}
void set_vert_orien()
{
  // Unlock configuration
  Serial1.write(unlock, 5);
  delay(1000);
  // Set to 200 HZ
  Serial1.write(set_vert, 5);
  delay(1000); // ensure set
  // Save configuration
  Serial1.write(save, 5);
  delay(1000);
}
void set_horiz_orien()
{
  // Unlock configuration
  Serial1.write(unlock, 5);
  delay(1000);
  // Set to 200 HZ
  Serial1.write(set_horiz, 5);
  delay(1000); // ensure set
  // Save configuration
  Serial1.write(save, 5);
  delay(1000);
}
void calib_gyro_and_accel()
{
  // Unlock configuration
  Serial1.write(unlock, 5);
  delay(1000);
  // calibrate gyro and accel mode
  Serial1.write(calib_gy_and_acc, 5);
  delay(6000); // ensure set
  // Save configuration
  Serial1.write(save, 5);
  delay(1000);
}

void calib_magnetometer()
{
  // Unlock configuration
  Serial1.write(unlock, 5);
  delay(1000);
  // calibrate the mag
  Serial1.write(calib_mag, 5);
  delay(5000); // ensure set
  // Save configuration
  Serial1.write(save, 5);
  delay(1000);
}

void set_height_0()
{
  // Unlock configuration
  Serial1.write(unlock, 5);
  delay(1000);
  // Set the z height to 0
  Serial1.write(calib_z_0, 5);
  delay(5000); // ensure set
  // Save configuration
  Serial1.write(save, 5);
  delay(1000);
}

void exit_calibration()
{
  // Unlock configuration
  Serial1.write(unlock, 5);
  delay(1000);
  // exit calibration
  Serial1.write(exit_calib, 5);
  delay(5000); // ensure set
  // Save configuration
  Serial1.write(save, 5);
  delay(1000);
}

void set_9axis()
{
  // Unlock configuration
  Serial1.write(unlock, 5);
  delay(1000);
  // exit calibration
  Serial1.write(set_9ax, 5);
  delay(5000); // ensure set
  // Save configuration
  Serial1.write(save, 5);
  delay(1000);
}

void set_get_only_ang_out()
{
  // Unlock configuration
  Serial1.write(unlock, 5);
  delay(1000);
  // exit calibration
  Serial1.write(set_ang_only_out, 5);
  delay(5000); // ensure set
  // Save configuration
  Serial1.write(save, 5);
  delay(1000);
}
void set_baud_rate()
{
  // Unlock configuration
  Serial1.write(unlock, 5);
  delay(1000);
  // exit calibration
  Serial1.write(set_baud, 5);
  delay(1000); // ensure set
  // Save configuration
  Serial1.write(save, 5);
  delay(1000);
  Serial1.write(unlock, 5);
  delay(1000);
  Serial1.write(save, 5);
  delay(1000);
  Serial1.write(save, 5);
  delay(1000);

}

// ================================================================
// ===                      Wifi Functions                     ===
// ================================================================
void setupWifi(){
  // Setting up the ESP32 Hosted wifi (IE you connect to a network being produced by the ESP32) 
  Serial.print("Setting AP (Access Point)…");
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: "); Serial.println(IP);

  // the address for this will be "http://192.168.4.1/
  //Endpoints control motor direction and autonomous behavior
  server.on("/forward",HTTP_POST,[](AsyncWebServerRequest * request){},
    NULL,[](AsyncWebServerRequest * request, uint8_t *data_in, size_t len, size_t index, size_t total) {
      
      //Don't accept commands unless autoOn is false
      Serial.println("Forward hit");
      if(!autoOn) moveForward(cruiseDutyCycle);
      request->send(200);
  });

  server.on("/backward",HTTP_POST,[](AsyncWebServerRequest * request){},
    NULL,[](AsyncWebServerRequest * request, uint8_t *data_in, size_t len, size_t index, size_t total) {

    //Don't accept commands unless autoOn is false
      Serial.println("Backward hit");
      if(!autoOn)  moveBackward(cruiseDutyCycle);
      request->send(200);
  });


  server.on("/right",HTTP_POST,[](AsyncWebServerRequest * request){},
    NULL,[](AsyncWebServerRequest * request, uint8_t *data_in, size_t len, size_t index, size_t total) {

    //Don't accept commands unless autoOn is false
      Serial.println("Right hit");
      if(!autoOn) turnRight(cruiseDutyCycle);
      request->send(200);
  });

  server.on("/left",HTTP_POST,[](AsyncWebServerRequest * request){},
    NULL,[](AsyncWebServerRequest * request, uint8_t *data_in, size_t len, size_t index, size_t total) {

    //Don't accept commands unless autoOn is false
      Serial.println("Left hit");
      if(!autoOn) turnLeft(cruiseDutyCycle);
      request->send(200);
  });

  server.on("/stop",HTTP_POST,[](AsyncWebServerRequest * request){},
    NULL,[](AsyncWebServerRequest * request, uint8_t *data_in, size_t len, size_t index, size_t total) {

    //Don't accept commands unless autoOn is false
      Serial.println("Stop hit");
      if(!autoOn) stopMotor();
      request->send(200);
  });

  server.on("/auto",HTTP_POST,[](AsyncWebServerRequest * request){},
    NULL,[](AsyncWebServerRequest * request, uint8_t *data_in, size_t len, size_t index, size_t total) {

    //Don't accept commands unless autoOn is false
      Serial.println("Auto hit");
      if(!autoOn) moveAuto();
      request->send(200);
  });
  // Start server (needed)
  server.begin();
}
