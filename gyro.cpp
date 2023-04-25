
#include "Gyro.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include "I2Cdev.h"
#include "Wire.h"
#define PI 3.1415926535897932384626433832795

Gyro :: Gyro(String url) {
  gyroURL = url;
  dmpReady = false;
  mpuInterrupt = false;
  interrupt_pin = 2;
}
void Gyro :: dmpDataReady() {
  return;
}
void Gyro :: create_json(char *tag, float *value, char *unit) {
  jsonDocument.clear();
  jsonDocument["type"] = tag;
  jsonDocument["euler_x"] = value[0];
  jsonDocument["euler_y"] = value[1];
  jsonDocument["euler_z"] = value[2];
  jsonDocument["unit"] = unit;
  serializeJson(jsonDocument, buffer);
}
void Gyro :: add_json_object(char *tag, float value, char *unit) {
  JsonObject obj = jsonDocument.createNestedObject();
  obj["type"] = tag;
  obj["value"] = value;
  obj["unit"] = unit;
}
char *Gyro :: getEuler() {
  create_json("euler_x", euler, "degrees");
  return buffer;
}

void Gyro :: sendGyro() {
  HTTPClient http;
  // Your Domain name with URL path or IP address with path
  http.begin(gyroURL);
  updateGyro();
  create_json("euler_x", euler, "degrees");
  http.addHeader("Content-Type", "application/json");
  int httpResponseCode = http.POST(buffer);
  http.end();
}

float Gyro :: getCurrentAngle() {
  return euler[0] * 180 / PI;
}

void Gyro :: gyroSetup() {
  Wire.begin(12, 13);
  mpu.initialize();
  pinMode(interrupt_pin, INPUT);
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(90);
  mpu.setYGyroOffset(44);
  mpu.setZGyroOffset(10);
  mpu.setZAccelOffset(1060); // 1688 factory default for my test chip
  mpu.setXAccelOffset(1491);
  mpu.setYAccelOffset(-5467);
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(10);
    mpu.CalibrateGyro(10);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it’s ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(interrupt_pin));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(interrupt_pin), dmpDataReady, RISING);
    mpuInterrupt = true;
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it’s okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it’s going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  // setup_task();
  mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetEuler(euler, &q);
  // euler_x = euler[0];
}
void Gyro :: updateGyro(){
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)){
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
  }
}