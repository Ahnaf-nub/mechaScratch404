float throttle = 0;
float targetThrottle = 0;
float steer = 0;
float targetSteer = 0;
float steerObstacle = 0;
float modAcrossX, modAcrossD;

int steerAngle = 32;
int steerMultiplier = -1; // set it to -1 if the steer is reversed

float throttleSmoothing = 0.6;
float steerSmoothing = 0.35;

int turnCount = 0;
int totalTurns = 12;
long lastTurnTimer;
int lastTurnTime = 6000;
bool finished = true;

int ignoreD = 10;
int rDmin[] = {5, 5, 8};    //Minimum distance range for {Side, Angle, Forward}
int rDmax[] = {80, 70, 20}; //Maximum distance range for {Side, Angle, Forward}

TaskHandle_t handleSonar;

// ---------------------------------- Husky ---------------------------------- //
#include "HUSKYLENS.h"
HUSKYLENS huskylens;

char objType = 'N'; // N = None, G = Green, R = Red
int objDist = 400;
int objSz = 0;
int objPos = -10;
int greenIDs[] = {1};
int redIDs[] = {2};

int xRange[] = {80, 240};
float rangeAcrossX[] = {0.6, 1};

int dRange[] = {15, 100};
float rangeAcrossD[] = {0.5, 1};
// ---------------------------------- Husky ---------------------------------- //
//                               ---------------                               //
// ---------------------------------- Sonar ---------------------------------- //
//#include <NewPing.h>

#define MAX_DISTANCE 120

//NewPing sonars[] = {
//  NewPing(26, 26, MAX_DISTANCE),
//  NewPing(27, 27, MAX_DISTANCE),
//  NewPing(32, 32, MAX_DISTANCE),
//  NewPing(33, 33, MAX_DISTANCE),
//  NewPing(25, 25, MAX_DISTANCE)
//};

int sPins[] = {26, 27, 32, 33, 25};
int dists[] = {0, 0, 0, 0, 0};
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
//                               ---------------                               //
// -------------------------------- INDICATOR -------------------------------- //
#define RED 15
#define GREEN 2
// -------------------------------- INDICATOR -------------------------------- //
//                               ---------------                               //
// -------------------------------- NEO_PIXEL -------------------------------- //
#include <Adafruit_NeoPixel.h>
#define PIN 5
#define NUMPIXELS 6
#define BRIGHTNESS 255
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
// -------------------------------- NEO_PIXEL -------------------------------- //


void setup() {
  Serial.begin(115200);

  xTaskCreatePinnedToCore(loopB,    "secondJob", 10000,   NULL,      1,        &handleSonar,            0);
  //                     (function, task name,   memory, parameter, priority, task reference,          core)

  pinMode(motPWM, OUTPUT);
  pinMode(motA, OUTPUT);
  pinMode(motB, OUTPUT);

  s.attach(servoPin);
  s.write(90);

  pinMode(btnPin, INPUT);

  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);

  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { }
  digitalWrite(GREEN, HIGH);

  while (!huskylens.begin(Wire)) {
    Serial.println(F("Begin failed!"));
    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(100);
  }
  digitalWrite(RED, HIGH);
  for (int i = 0; i < sizeof(greenIDs) / sizeof(greenIDs[0]); i++) {
    huskylens.setCustomName("Green", greenIDs[i]);
  }
  for (int i = 0; i < sizeof(redIDs) / sizeof(redIDs[0]); i++) {
    huskylens.setCustomName("Red", redIDs[i]);
  }

  pixels.begin();
  pixels.clear();
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(BRIGHTNESS, BRIGHTNESS, (BRIGHTNESS * 2) / 3));
    pixels.show();
  }

  Serial.println(F("Calculating offsets"));
  // mpu.upsideDownMounting = true;
  mpu.calcOffsets();
  Serial.println("Done!\n");
  while (digitalRead(btnPin) == 0) { } // wait untill the button is pressed
  delay(1000);
  digitalWrite(GREEN, LOW);
  digitalWrite(RED, LOW);
  finished = false;
  currentAngle = mpu.getAngleZ();
  lastAngle = currentAngle;
}

void loop() {
  mpu.update();
  currentAngle = mpu.getAngleZ();
  if (abs(lastAngle - currentAngle) > 90) {
    turnCount ++;
    lastAngle = currentAngle;
  }

  if (!finished && turnCount >= totalTurns) { // if the car finishes three laps
    finished = true;
    lastTurnTimer = millis();
  }

  setControls();
  modifyControlsObstacles();

  targetSteer += steerObstacle;
  targetThrottle = clamp(targetThrottle, 0, 1);
  targetSteer = clamp(targetSteer, -1, 1);

  throttle = lerpF(throttle, targetThrottle, throttleSmoothing);
  steer = lerpF(steer, targetSteer, steerSmoothing);

  avoidCollision();

  setThrottleSteer(throttle, steer);

  if (finished && millis() - lastTurnTimer > lastTurnTime) { // Run another <lastTurnTime> milliseconds
    setThrottleSteer(0, 0); // stop the car
    digitalWrite(GREEN, HIGH);
    while (digitalRead(btnPin) == 0) { } // wait untill the button is pressed
    delay(1000);

    currentAngle = mpu.getAngleZ();
    lastAngle = currentAngle;
    turnCount = 0;
    finished = false;
  }

  delay(10);
}

void loopB(void * param) {
  while (true) {
    getDistanceValues();
    getHuskyData();

    if (finished) {
      modifyControlsObstacles();
    }
    huskylens.customText(String(modAcrossX), 10, 60);
    huskylens.customText(String(modAcrossD), 10, 90);
    huskylens.customText(String(steerObstacle), 60, 75);

    huskylens.customText(String(throttle) + " | " + String(steer), 195, 10);
  }
}

void getHuskyData() {
  objPos = -10;
  objDist = 400;
  objSz = 0;
  objType = 'N';

  if (!huskylens.request()) {
    huskylens.customText("- Request Failed -", 10, 10);
  }
  else if (!huskylens.available()) {
    huskylens.customText("- Nothing Available -", 10, 10);
  }
  else
  {
    while (huskylens.available())
    {
      HUSKYLENSResult result = huskylens.read();
      int _objDist = 700 / result.width;
      if (_objDist < objDist) {
        objDist = _objDist;
        objSz = result.width / 2;
        objPos = result.xCenter;
        if (contains(greenIDs, result.ID)) {
          objType = 'G';
        }
        else if (contains(redIDs, result.ID)) {
          objType = 'R';
        }
      }
    }
  }
  if (objType != 'N') {
    huskylens.customText(objType + " " + String(objPos) + " | " + String(objDist), 10, 10);
  } else {
    huskylens.customText("- All Clear -", 10, 10);
  }
}


void modifyControlsObstacles() {
  for (int i = 0; i < 5; i++) {
    if (i != 2 && dists[i] < ignoreD) {
      return;
    }
  }
  modAcrossX = 0;
  modAcrossD = 0;
  if (objType != 'N') {
    modAcrossD = mapFC(objDist, dRange[0], dRange[1], rangeAcrossD[1], rangeAcrossD[0]);
    if (objType == 'R') {
      modAcrossX =  mapFC(objPos, xRange[0] + objSz, xRange[1] - objSz, rangeAcrossX[0], rangeAcrossX[1]);
    }
    else {
      modAcrossX = -mapFC(objPos, xRange[0] + objSz, xRange[1] - objSz, rangeAcrossX[1], rangeAcrossX[0]);
    }
  }

  steerObstacle = modAcrossX * modAcrossD;
}

void setControls() {
  float _thr = 1;
  float _steer = 0;

  if (dists[0] < rDmax[0]) {
    _steer -= mapFC(dists[0], rDmin[0], rDmax[0], 0.5, 0);
  }
  if (dists[1] < rDmax[1]) {
    _steer -= mapFC(dists[1], rDmin[1], rDmax[1], 1, 0) * 1;
    //_thr -= mapFC(dists[1], rDmin[1], rDmax[1], 0.75, 0);
  }

  if (dists[3] < rDmax[1]) {
    _steer += mapFC(dists[3], rDmin[1], rDmax[1], 1, 0)  * 1;
    //_thr -= mapFC(dists[3], rDmin[1], rDmax[1], 0.75, 0);
  }
  if (dists[4] < rDmax[0]) {
    _steer += mapFC(dists[4], rDmin[0], rDmax[0], 0.5, 0);
  }

  targetThrottle = _thr;
  targetSteer = _steer;
}

void getDistanceValues() {
  long sonarTimer = millis();

  Serial.print("Sonar Values: ");
  for (int i = 0; i < 5; i++) {
    dists[i] = getDistS(sPins[i]);
    Serial.print(dists[i]);
    Serial.print(" - ");
  }
  Serial.println("");
  delay(10);

  huskylens.customText(S(dists[4]) + "|" + S(dists[3]) + "|" + S(dists[2]) + "|"  + S(dists[1]) + "|"  + S(dists[0]), 10, 200);

  while (millis() - sonarTimer < 15) { } // ensures a 15ms delay between pings per sonar
}

int getDistS(int te) {
  pinMode(te, OUTPUT);

  digitalWrite(te, LOW);
  delayMicroseconds(2);
  digitalWrite(te, HIGH);
  delayMicroseconds(10);
  digitalWrite(te, LOW);

  pinMode(te, INPUT);

  long dur = pulseIn(te, HIGH, 7200);
  long dst = dur / 58.8;
  if (dst == 0) dst = MAX_DISTANCE;
  return (dst);
}

void avoidCollision() {
  if (dists[2] < rDmax[2]) {
    targetThrottle -= mapFC(dists[2], rDmin[2], rDmax[2], 1, 0.3);
    targetSteer *= mapFC(dists[2], rDmin[2], rDmax[2], 1, 1.5);
  }

  if (dists[2] < rDmin[2]) {
    long _tmr = millis();
    float _str  = steer * 5;
    if (objType == 'G' && objDist < 15) {
      _str = 0.5;
    }
    else if (objType == 'R' && objDist < 15) {
      _str = -0.5;
    }
    while (millis() - _tmr < 1150) {
      setThrottleSteer(-0.75, _str);
      mpu.update();
    }
  }
}

void setThrottleSteer(float _thr, float _str) {
  _thr = clamp(_thr, -1, 1);
  _str = clamp(_str, -1, 1);
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

String S(int val) {
  String ret = String(val);
  if (ret.length() == 1) {
    return ("00" + ret);
  }
  if (ret.length() == 2) {
    return ("0" + ret);
  }
  return (ret);
}

bool contains(int arr[], int val) {
  if (sizeof(arr) == 0) {
    return false;
  }

  for (int i = 0; i < sizeof(arr) / sizeof(arr[0]); i++) {
    if (arr[i] == val) {
      return true;
    }
  }

  return false;
}

float mapF(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float mapFC(float x, float in_min, float in_max, float out_min, float out_max) {
  return clamp( mapF(x, in_min, in_max, out_min, out_max), out_min, out_max);
}

float clamp(float val, float mini, float maxi) {
  float tMin = mini;
  float tMax = maxi;
  if (mini > maxi) {
    tMin = maxi;
    tMax = mini;
  }
  if (val < tMin) {
    return tMin;
  }
  if (val > tMax) {
    return tMax;
  }
  return val;
}

float lerpF(float a, float b, float t)
{
  return a + t * (b - a);
}
