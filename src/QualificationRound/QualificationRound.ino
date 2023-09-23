float throttle = 0;
float steer = 0;
int steerAngle = 45;
int steerMultiplier = 1; // set it to -1 if the steer is reversed

float throttleSmoothing = 0.6;
float steerSmoothing = 0.5;

int turnCount = 0;
int totalTurns = 12;
long lastTurnTimer;
int lastTurnTime = 6000;
boolean finished = false;

long rDmin[] = {2, 2, 2};    //Minimum distance range for {Side, Angle, Forward}
long rDmax[] = {80, 30, 30}; //Maximum distance range for {Side, Angle, Forward}

TaskHandle_t handleSonar;

// ---------------------------------- Sonar ---------------------------------- //
#include <NewPing.h>

#define MAX_DISTANCE 120

NewPing sonars[] = {
  NewPing(34, 34, MAX_DISTANCE),
  NewPing(35, 35, MAX_DISTANCE),
  NewPing(32, 32, MAX_DISTANCE),
  NewPing(33, 33, MAX_DISTANCE),
  NewPing(25, 25, MAX_DISTANCE)
};

long dists[] = {0, 0, 0, 0, 0};
// ---------------------------------- Sonar ---------------------------------- //
//                               ---------------                               //
// ---------------------------------- Motor ---------------------------------- //
#define motPWM 4
#define motA 16
#define motB 17
// ---------------------------------- Motor ---------------------------------- //
//                               ---------------                               //
// ---------------------------------- Servo ---------------------------------- //
#include <ESP32Servo.h>

Servo s;
#define servoPin 18
// ---------------------------------- Servo ---------------------------------- //
//                               ---------------                               //
// ----------------------------------- Btn ----------------------------------- //
#define btnPin 19
// ----------------------------------- Btn ----------------------------------- //
//                               ---------------                               //
// ----------------------------------- MPU ----------------------------------- //
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
long lastAngle;
long currentAngle;
// ----------------------------------- MPU ----------------------------------- //

void setup() {
  Serial.begin(115200);

  xTaskCreatePinnedToCore(loopB,    "secondJob", 1024,   NULL,      1,        &handleSonar,            0);
  //                     (function, task name,   memory, parameter, priority, task reference,          core)

  pinMode(motPWM, OUTPUT);
  pinMode(motA, OUTPUT);
  pinMode(motB, OUTPUT);

  s.attach(servoPin);
  s.write(90);

  pinMode(btnPin, INPUT_PULLUP);


  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { }

  Serial.println(F("Calculating offsets"));
  // mpu.upsideDownMounting = true;
  mpu.calcOffsets();
  Serial.println("Done!\n");

  while (digitalRead(btnPin) == 1) { } // wait untill the button is pressed
  delay(1000);

  currentAngle = mpu.getAngleZ();
  lastAngle = currentAngle;
}

void loop() {
  mpu.update();
  currentAngle = mpu.getAngleZ();
  if (abs(lastAngle - currentAngle) > 80) {
    turnCount ++;
    lastAngle = currentAngle;
  }
  if (!finished && turnCount >= totalTurns) { // if the car finishes three laps
    finished = true;
    lastTurnTimer = millis();
  }

  setControls();
  throttle = clamp(throttle, 0, 1);
  steer = clamp(steer, -1, 1);
  setThrottleSteer(throttle, steer);

  if (finished && millis() - lastTurnTimer > lastTurnTime) { // Run another <lastTurnTime> milliseconds
    setThrottleSteer(0, 0); // stop the car
    while (digitalRead(btnPin) == 1) { } // wait untill the button is pressed
    delay(1000);

    currentAngle = mpu.getAngleZ();
    lastAngle = currentAngle;
    turnCount = 0;
    finished = false;
  }
}

void loopB(void * param) {
  while (true) {
    getDistanceValues();
  }
}


void setControls() {
  float _thr = 1;
  float _steer = 0;

  if (dists[0] < rDmax[0]) {
    _steer -= mapFC(dists[0], rDmin[0], rDmax[0], 0.8, 0);
  }
  if (dists[1] < rDmax[1]) {
    _steer -= mapFC(dists[1], rDmin[1], rDmax[1], 1, 0);
    _thr -= mapFC(dists[1], rDmin[1], rDmax[1], 0.75, 0);
  }

  if (dists[2] < rDmax[2]) {
    _thr -= mapFC(dists[2], rDmin[2], rDmax[2], 1, 0);
  }

  if (dists[3] < rDmax[1]) {
    _steer += mapFC(dists[3], rDmin[1], rDmax[1], 1, 0);
    _thr -= mapFC(dists[3], rDmin[1], rDmax[1], 0.75, 0);
  }
  if (dists[4] < rDmax[0]) {
    _steer += mapFC(dists[4], rDmin[0], rDmax[0], 0.8, 0);
  }

  throttle = lerpF(throttle, _thr, throttleSmoothing);
  steer = lerpF(steer, _steer, steerSmoothing);
}

void getDistanceValues() {
  long sonarTimer = millis();

  for (int i = 0; i < 5; i++) {
    dists[i] = sonars[i].ping_cm();
  }

  while (millis() - sonarTimer < 30) { } // ensures a 30ms delay between pings per sonar
}

void setThrottleSteer(float _thr, float _str) {
  if (_thr == 0) {
    digitalWrite(motA, LOW);
    digitalWrite(motB, LOW);
  }
  else if (_thr > 0) {
    digitalWrite(motA, HIGH);
    digitalWrite(motB, LOW);
    analogWrite(motPWM, int(_thr * 255));
  }
  else {
    digitalWrite(motA, LOW);
    digitalWrite(motB, HIGH);
    analogWrite(motPWM, int(-_thr * 255));
  }

  s.write(90 + _str * steerAngle * steerMultiplier);
}

float mapF(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float mapFC(float x, float in_min, float in_max, float out_min, float out_max) {
  return clamp( mapF(x, in_min, in_max, out_min, out_max), out_min, out_max);
}

float clamp(float val, float mini, float maxi) {
  if (val < mini) {
    return mini;
  }
  if (val > maxi) {
    return maxi;
  }
  return val;
}

float lerpF(float a, float b, float t) {
  return a + t * (b - a);
}
