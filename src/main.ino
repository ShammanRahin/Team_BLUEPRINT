#include <Wire.h>
#include "VL53L1X_ULD.h"
#include <Servo.h>
#include "Adafruit_TCS34725.h"

#define TCA_ADDR 0x70   // I2C multiplexer address
#define RPWM 3
#define LPWM 11

Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_154MS, 
  TCS34725_GAIN_16X
);
const int TRIG_PIN = 12;
const int ECHO_PIN = 10;

// Anything over 400 cm (23200 us pulse) is "out of range"
const unsigned int MAX_DIST = 23200;

int colorVal = 2;   // 1=red, 0=yellow, 2=none
int errorVal = 0;   // horizontal offset (pixels)
int areaVal  = 0; 
int lastLidar1 = 0;
int lastLidar2 = 0;
Servo myServo;
VL53L1X_ULD lidar1;
VL53L1X_ULD lidar2;

int prevAngle = 97;
uint16_t r, g, b, c;
float R,G,B ;

// ------------------ Multiplexer ------------------
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delayMicroseconds(500);
}
  int ReadSonar(){
  long duration;
  int distance;

  // Trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo
  duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout 30 ms (≈5 m range)

  // Convert to cm
  distance = duration * 0.034 / 2; 

  if (duration == 0) {
    Serial.println("Out of range");
  } else {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
  return distance;
  }
// ------------------ Lidar Functions ------------------
bool initLidar(VL53L1X_ULD &sensor, uint8_t roiX, uint8_t roiY, uint16_t centerSPAD) {
  VL53L1_Error status = sensor.Begin();
  if (status != VL53L1_ERROR_NONE) {
    Serial.print("Sensor init failed, error: ");
    Serial.println(status);
    return false;
  }
  sensor.SetTimingBudgetInMs(50); 
  sensor.SetInterMeasurementInMs(50);
  sensor.SetDistanceMode(Long);
  sensor.StartRanging();
  return true;
}

int readLidar(VL53L1X_ULD &sensor, int &lastValid) {
  uint8_t dataReady = 0;
  sensor.CheckForDataReady(&dataReady);

  if (dataReady) {
    uint16_t distance;
    VL53L1_Error status = sensor.GetDistanceInMm(&distance);
    sensor.ClearInterrupt();

    if (status == VL53L1_ERROR_NONE && distance > 0 && distance < 4000) {
      lastValid = distance;   // update stored value
    }
  }

  return lastValid;  // always return the most recent valid reading
}

// ------------------ Setup ------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);

  myServo.attach(9);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  myServo.write(prevAngle);
  delay(500);

  tcaSelect(2);
  if (!initLidar(lidar1, 16, 14, 199)) {
    Serial.println("Failed to init lidar1");
    while(1);
  }
   
    // Initialize color sensor on TCA channel 4
  tcaSelect(4);
  delay(5);
  if (!tcs.begin()) {
    Serial.println("TCS34725 not found on channel 4!");
    // Continue anyway, readColor will fail-safe
  } else {
    Serial.println("TCS34725 initialized on channel 4");
  }

  tcaSelect(6);
  if (!initLidar(lidar2, 16, 14, 199)) {
    Serial.println("Failed to init lidar2");
    while(1);
  } 

  Serial.println("Both lidars initialized with ROI!");
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
}
int readColor() {
  // Select TCS34725 channel
  tcaSelect(4);
  delay(2);  // small mux settle delay

  // Get raw RGB+clear values
  tcs.getRawData(&r, &g, &b, &c);

  if (c == 0) return -1; // avoid divide-by-zero

  // Normalize (scale to 0–255 range)
  R = (float)r / c * 255.0;
  G = (float)g / c * 255.0;
  B = (float)b / c * 255.0;

  Serial.print("R: "); Serial.print(R);
  Serial.print(" G: "); Serial.print(G);
  Serial.print(" B: "); Serial.print(B);
  Serial.print(" C: "); Serial.println(c);
  }

// ------------------ Loop ------------------
void loop() {
  // --- Read lidars ---
  tcaSelect(2);
  int d2 = readLidar(lidar1, lastLidar1) / 10;   // left lidar

  tcaSelect(6);
  int d1 = readLidar(lidar2, lastLidar2) / 10;   // right lidar

  Serial.print("Right Lidar: "); Serial.print(d1); Serial.print(" cm | ");
  Serial.print("Left Lidar: "); Serial.println(d2); Serial.println(" cm");

  if (d1 < 0 || d2 < 0) return;

  int wallError = d1 - d2;  // right-left difference
  int front = ReadSonar();

  // --- Update vision data ---
  ReadSerial();

  // --- Combined PD control (wall + pillar avoidance) ---
  int servoAngle = CombinedControl(wallError , front);
    if ( front < 30){ 
      servoAngle = servoAngle < 97 ? 127 : 67 ;
      myServo.write(servoAngle);
      analogWrite(RPWM, 0);
      analogWrite(LPWM, 100);
      delay(500);
  }
  else {
  // --- Drive ---
  myServo.write(servoAngle);
  analogWrite(RPWM, 80);
  analogWrite(LPWM, 0);
  }

  
  Serial.print("Servo: "); Serial.println(servoAngle);
}

// ------------------ Serial Read ------------------
void ReadSerial() {
  if (Serial.available() >= 3) {
    colorVal = Serial.parseInt();
    errorVal = Serial.parseInt();
    areaVal  = Serial.parseInt();
    while (Serial.available() > 0) Serial.read(); // clear buffer
  }
}

// ------------------ Combined Control ------------------
int CombinedControl(int wallError , int front) {
  // --- Wall PD ---
  static int prevWallError = 0;
  static unsigned long prevTime = 0;
  float Kp_wall = 2;
  float Kd_wall = 0.5;

  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;
  if (dt <= 0) dt = 0.001;

  float P_wall = Kp_wall * wallError;
  float D_wall = Kd_wall * (wallError - prevWallError) / dt;
  float wallOutput = P_wall + D_wall;

  prevWallError = wallError;
  prevTime = now;

  // --- Vision desired offset ---
int desiredOffset = 0;
if (colorVal == 1) desiredOffset = -100 ; // red
else if (colorVal == 0) desiredOffset = 100 ; // yellow

  int visError = errorVal - desiredOffset;
  float Kp_vis = 0.9;
  float Kd_vis = 0.05;
  static int prevVisError = 0;
  static float prevVisDeriv = 0;

  float D_vis = Kd_vis * (visError - prevVisError) / dt;
  float visOutput = Kp_vis * visError + D_vis;

  prevVisError = visError;

  // --- Blending ---
  float w_vis = 0.0;
if (areaVal > 300) {
    w_vis = (float)(areaVal - 1000) / (1800 - 1000);  // linear map from 300–2300
    w_vis = constrain(w_vis, 0.0, 1.0);             // make sure it stays 0–1
} else {
    w_vis = 0.0;
}

  float blended = (1 - w_vis) * wallOutput + w_vis * visOutput;

  // --- Map to servo ---
  int servoAngle = 97 - (int)blended;
  servoAngle = constrain(servoAngle, 67, 127);
    if(front < 50){
    if(wallError < 0){
      servoAngle = 127;
    }
    if(wallError> 0){
      servoAngle = 67;
    }
  }
  return servoAngle;
}