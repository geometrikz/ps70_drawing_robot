#include "Driving.h"
#include <Arduino.h>
#include "Gyro.h"
#include <AccelStepper.h>
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <ArduinoJson.h>

#define PI 3.1415926535897932384626433832795
// #define WHEEL_RADIUS 34.0
#define WHEEL_RADIUS 40.0
#define TURNING_RADIUS 136

Driving :: Driving(int pin1, int pin2, int pin3, int pin4, int maxSp, int maxAc, int motorSPR, String url) {
  myGyro = Gyro(url);
  stepper1 = AccelStepper(AccelStepper::DRIVER, pin1, pin2);
  stepper2 = AccelStepper(AccelStepper::DRIVER, pin3, pin4);
  maxSpeed = maxSp;
  maxAccel = maxAc;
  motorStepsPerRevolution = motorSPR;
}

void Driving :: motorReset() {
  stepper1.setMaxSpeed(maxSpeed);
  stepper1.setAcceleration(maxAccel);
  stepper2.setMaxSpeed(maxSpeed);
  stepper2.setAcceleration(maxAccel);
}

void Driving :: setup() {
  motorReset();
  myGyro.gyroSetup();
}

// Converts machine turning degrees to motor degrees
float Driving :: machineToMotor(float machine_degrees) {
    return machine_degrees * TURNING_RADIUS / (WHEEL_RADIUS);
}

// Converts forward distance (in cm) to motor degrees
float Driving :: forwardToMotor(float forward_cm) {
  return forward_cm * 180 / (PI * WHEEL_RADIUS);
}

// Loops through to execute whatever commands are sent to the two motors
void Driving :: execute(float deg1, float deg2) {
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  delay(100);
  stepper1.moveTo(deg1); 
  stepper2.moveTo(deg2);
  while (stepper1.distanceToGo() != 0 && stepper2.distanceToGo() != 0) {
    stepper1.run(); 
    stepper2.run();
    // myGyro.updateGyro();
    // myGyro.sendGyro();
  }
}

// moves machine forward x centimeters
void Driving :: move_forward(float forward_cm) {
  float degrees = forwardToMotor(forward_cm) * motorStepsPerRevolution/360;
  execute(degrees, degrees);
}

// Turns machine X degrees
void Driving :: turn(float machine_degrees) {
  float degrees = machineToMotor(machine_degrees) * motorStepsPerRevolution/360;

  // set up gyro
  float current_degrees = myGyro.getCurrentAngle();
  execute(-1*degrees, degrees);

  // Uncomment to act on degree change.
  // need to worry about measurements getting cut off and also direction.
  // float degree_change = myGyro.getCurrentDegrees() - current_degrees;
  // if (abs(machine_degrees - degree_change) > 5) {
  //   turn(machine_degrees - degree_change);
  // }
}

void Driving :: draw_circle(float diameter) {
  // Go to outside of 
  float max_speed = 150;
  float max_accel = 4000;
  float inner_distance = 1.9 *PI * (diameter - TURNING_RADIUS*2); // no /2
  float outer_distance = 1.9 *PI * (diameter + TURNING_RADIUS*2);
  float inner_val = inner_distance / (inner_distance + outer_distance);
  float outer_val = outer_distance / (inner_distance + outer_distance);

  stepper1.setMaxSpeed(max_speed * inner_val);
  stepper1.setAcceleration(max_accel*inner_val);
  stepper2.setMaxSpeed(max_speed * outer_val);
  stepper2.setAcceleration(max_accel * outer_val);

  float degrees1 = forwardToMotor(inner_distance) * motorStepsPerRevolution/360;
  float degrees2 = forwardToMotor(outer_distance) * motorStepsPerRevolution/360;
  execute(degrees2, degrees1);

}

void Driving :: sendGyro(){
  myGyro.sendGyro();
}
