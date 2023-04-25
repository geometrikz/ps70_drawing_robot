#include <AccelStepper.h>
#include <WiFi.h>
#include "gyro.h"
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "Driving.h"
// #include <Wire.h>

#define MAX_ACCELERATION 50
#define MAX_SPEED 200
#define WATER_PIN 40
#define INTERRUPT_PIN 2

// Connect to WiFi
const char* ssid = "MAKERSPACE";
const char* password = "12345678";
bool first_read = true;
int id;

// Set up server for sending Gyro Data and reading instructions
// const String apiURL = "http://192.168.0.117:3000/api_coordinates";
// const String gyroURL = "http://192.168.0.117:3000/gyro";
const String apiURL = "https://ps70-api.vercel.app/";
const String gyroURL = "https://ps70-api.vercel.app/gyro";


//Gyro mpu = Gyro(gyroURL);

// MPU6050 mpu;
// // Set up inputs for gyroscope
// bool dmpReady = false;  // set true if DMP init was successful
// uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
// uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
// uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
// uint16_t fifoCount;     // count of all bytes currently in FIFO
// uint8_t fifoBuffer[64]; // FIFO storage buffer
// Quaternion q;           // [w, x, y, z]         quaternion container
// VectorInt16 aa;         // [x, y, z]            accel sensor measurements
// VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
// VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
// VectorFloat gravity;    // [x, y, z]            gravity vector
// float euler[3];         // [psi, theta, phi]    Euler angle container
// float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
// void dmpDataReady() {
//     mpuInterrupt = true;
// }

// StaticJsonDocument<250> jsonDocument;
// char buffer[250];
// void create_json(char *tag, float *value, char *unit) {  
//   jsonDocument.clear();
//   jsonDocument["type"] = tag;
//   jsonDocument["euler_x"] = value[0];
//   jsonDocument["euler_y"] = value[1];
//   jsonDocument["euler_z"] = value[2];
//   jsonDocument["unit"] = unit;
//   serializeJson(jsonDocument, buffer);  
// }

// void add_json_object(char *tag, float value, char *unit) {
//   JsonObject obj = jsonDocument.createNestedObject();
//   obj["type"] = tag;
//   obj["value"] = value;
//   obj["unit"] = unit; 
// }

// void sendGyro() {
//   HTTPClient http;
//   // Your Domain name with URL path or IP address with path
//   http.begin(gyroURL);
//   updateGyro();
//   create_json("euler_x", euler, "degrees");
//   http.addHeader("Content-Type", "application/json");
//   int httpResponseCode = http.POST(buffer);
//   http.end();
// }

// Define motor pin connections
const int motorPin1 = 11;
const int motorPin2 = 10;
const int motorPin3 = 7;
const int motorPin4 = 6;

// Define the motor steps per revolution
const int motorStepsPerRevolution = 200;
long old_id = -1;

Driving driver = Driving(motorPin1, motorPin2, motorPin3, motorPin4, MAX_SPEED, MAX_ACCELERATION, motorStepsPerRevolution, gyroURL);

// void gyroSetup() {
//   // Set max speed and acceleration
//   Wire.begin(12, 13);
//   mpu.initialize();
//   pinMode(INTERRUPT_PIN, INPUT);
//   devStatus = mpu.dmpInitialize();
//   mpu.setXGyroOffset(90);
//   mpu.setYGyroOffset(44);
//   mpu.setZGyroOffset(10);
//   mpu.setZAccelOffset(1060); // 1688 factory default for my test chip
//   mpu.setXAccelOffset(1491);
//   mpu.setYAccelOffset(-5467);
//   if (devStatus == 0) {
//     // Calibration Time: generate offsets and calibrate our MPU6050
//     mpu.CalibrateAccel(10);
//     mpu.CalibrateGyro(10);
//     mpu.PrintActiveOffsets();
//     // turn on the DMP, now that it's ready
//     Serial.println(F("Enabling DMP..."));
//     mpu.setDMPEnabled(true);

//     // enable Arduino interrupt detection
//     Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
//     Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
//     Serial.println(F(")..."));
//     attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
//     mpuIntStatus = mpu.getIntStatus();

//     // set our DMP Ready flag so the main loop() function knows it's okay to use it
//     Serial.println(F("DMP ready! Waiting for first interrupt..."));
//     dmpReady = true;

//     // get expected DMP packet size for later comparison
//     packetSize = mpu.dmpGetFIFOPacketSize();
//   } else {
//     // ERROR!
//     // 1 = initial memory load failed
//     // 2 = DMP configuration updates failed
//     // (if it's going to break, usually the code will be 1)
//     Serial.print(F("DMP Initialization failed (code "));
//     Serial.print(devStatus);
//     Serial.println(F(")"));
//   }

//   mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
//   mpu.dmpGetQuaternion(&q, fifoBuffer);
//   mpu.dmpGetEuler(euler, &q);
// }

void setup() {
  // Set max speed and acceleration
  driver.setup();

  // Set up water pump pin
  pinMode(WATER_PIN, OUTPUT);
  Serial.begin(115200);

  // Set up WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi");
  }
  Serial.println("Connected to the WiFi network");

  // Setup gyro
  // mpu.gyroSetup();
  //gyroSetup();
}

void toggle_pump(int on) {
  if (on) {
    digitalWrite(WATER_PIN, LOW);
  } else {
    digitalWrite(WATER_PIN, HIGH);
  }
}

// void updateGyro(){
//   if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)){
//     mpu.dmpGetQuaternion(&q, fifoBuffer);
//     mpu.dmpGetEuler(euler, &q);
//   }
// }

void draw_circle(float diameter) {
  toggle_pump(1);
  delay(300);
  toggle_pump(0);
  driver.move_forward(diameter / 2);
  driver.turn(90);
  toggle_pump(1);
  driver.draw_circle(diameter);
  toggle_pump(0);
}


void loop() {
  // Serial.println("Refreshing");
  if ((WiFi.status() == WL_CONNECTED)) {
    HTTPClient http;
    http.begin(apiURL);
    int httpResponseCode = http.GET();
    if (httpResponseCode > 0) {
      String payload = http.getString();
      DynamicJsonDocument doc(8192);
      DeserializationError error = deserializeJson(doc, payload);
      if (error) {
        Serial.println("Deserialization Error");
        return;
      }
      http.end();
      driver.sendGyro();

      id = doc["id"];
      
      if (id != old_id && !first_read) {
        if (id > 0) {
          for (JsonObject instruction : doc["instructions"].as<JsonArray>()) {
            double instruction_angle = instruction["angle"];
            double instruction_distance = instruction["distance"];
            int instruction_pendown = instruction["pendown"];
            toggle_pump(0);
            driver.turn(instruction_angle);
            toggle_pump(instruction_pendown);
            driver.move_forward(instruction_distance);
          }
        } else {
          toggle_pump(1);
          draw_circle(1000);
          toggle_pump(0);
        }
      old_id = id;
      toggle_pump(0);
    }
    // old_id = id;
    first_read = false;
    driver.setup();
  }
}
}