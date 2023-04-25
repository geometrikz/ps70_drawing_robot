#ifndef GYRO_H
#define GYRO_H
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WebServer.h>
// #include "MPU6050.h"
#include "helper_3dmath.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

class Gyro
{
  private:
    MPU6050 mpu;
    bool dmpReady;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float euler[3];         // [psi, theta, phi]    Euler angle container
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    volatile bool mpuInterrupt; 
    StaticJsonDocument<250> jsonDocument;
    char buffer[250];
    int interrupt_pin;
    String gyroURL;

  public:
    Gyro(String url);
    static void dmpDataReady();
    void create_json(char *tag, float *value, char *unit);
    void add_json_object(char *tag, float value, char *unit);
    char *getEuler();
    void gyroSetup();
    void updateGyro();
    void sendGyro();
    float getCurrentAngle();
};

#endif