#include <Arduino.h>
#include <math.h>


#include <Wire.h>               // For I2C communication
#include <Adafruit_GFX.h>       // Core graphics library
#include <Adafruit_SSD1306.h>   // Hardware-specific library for SSD1306

#define SCREEN_WIDTH 128        // OLED display width, in pixels
#define SCREEN_HEIGHT 64        // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The -1 indicates the display does not have a dedicated reset pin
#define OLED_RESET -1 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define MUX_S0 8
#define MUX_S1 10
#define MUX_S2 11
#define MUX_S3 12
#define MUX_SIG A7

// Left motor
#define ENA 9
#define IN1 6
#define IN2 4

// Right motor
#define ENB 5
#define IN3 3
#define IN4  2

float Kp = 100.0;
float Ki = 0.0;
float Kd = 12.0;

#define NUM_SENSORS 14
struct Rot{
  int turn;
  unsigned int time;
  Rot(char t , unsigned int time){
    
  }

};
float error = 0, lastError = 0;
float integral = 0;
float derivative = 0;

int baseSpeed = 50;
int maxSpeed  = 255;

unsigned int sensorBits = 0;
int threshold = 500;
unsigned long lastPIDTime = 0;
unsigned long lastJunctionTime = 0;
const unsigned long JUNCTION_DEBOUNCE = 300; // ms
bool isInverse=false;

void selectMuxChannel(int ch) {
  digitalWrite(MUX_S0, bitRead(ch, 0));
  digitalWrite(MUX_S1, bitRead(ch, 1));
  digitalWrite(MUX_S2, bitRead(ch, 2));
  digitalWrite(MUX_S3, bitRead(ch, 3));
}

void readSensorsAnalog() {
  sensorBits = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    selectMuxChannel(i);
    delayMicroseconds(5);
    int sensorVal = analogRead(MUX_SIG);
    Serial.print(sensorVal);
    Serial.print(" ");
    
  }
  Serial.println();
}

int readSensors(bool isInverse) {
  sensorBits = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    selectMuxChannel(i);
    delayMicroseconds(5);
    int sensorVal = analogRead(MUX_SIG);
    if (sensorVal > threshold) {
      sensorBits |= (1 << i);
    }
  }
  if (isInverse) {
  sensorBits = (~sensorBits) & 0x3FFF;  // keep only 14 bits
}
  return sensorBits;
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, constrain(leftSpeed, 0, 255));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, constrain(-leftSpeed, 0, 255));
  }

  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, constrain(rightSpeed, 0, 255));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, constrain(-rightSpeed, 0, 255));
  }
}
int calculatePID(unsigned int sensorBits) {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0;
  lastPIDTime = now;
  if (dt <= 0) dt = 0.01;
  long weightedSum = 0;
  int sensorCount = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int bitVal = (sensorBits >> i) & 1;
    long weight = i * 1000;
    weightedSum += bitVal * weight;
    sensorCount += bitVal;
  }

  if (sensorCount == 0) return 0;

  float rawError = (float)weightedSum / sensorCount;

  float mappedError = (rawError - 6500) / 6500;
  mappedError = constrain(mappedError, -1.0, 1.0);
  error = mappedError;
  integral += error * dt;
  derivative = (error - lastError) / dt;

  lastError = error;

  return 1;
}
int detectJunction(unsigned int sensorBits) {
  unsigned long currentTime = millis();
  
  // 1. Check if we are still in the "quiet period"
  if (currentTime - lastJunctionTime < JUNCTION_DEBOUNCE) {
    return 0; 
  }

  // 2. Check for the pattern
  if(sensorBits == 0b11111100111111 || 
     sensorBits == 0b11111001111111 || 
     sensorBits == 0b11111110011111) {
    
    lastJunctionTime = currentTime; 
    return 1;
  }
  
  return 0;
}

void printBinary(unsigned int n) {
    int bits = sizeof(unsigned int) * 8;
    for (int i = bits - 1; i >= 0; i--) {
        Serial.print((n >> i) & 1);
    }
    Serial.println();
}

void displayJunction(unsigned int pattern, const char* str = "Nothing") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Print Label Header
  display.setCursor(0, 5);
  display.print(F("Junction Status:"));

  // Print 14-bit Binary Pattern
  display.setCursor(0, 20);
  for (int i = 13; i >= 0; i--) {
    display.print((pattern >> i) & 1);
  }

  // Print the passed string (e.g., "Inverse ON")
  display.setCursor(0, 40);
  display.print(F("Mode: "));
  display.print(str);

  display.display();
}
void setup() {
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    // If initialization fails, the Nano will stop here (infinite loop)
    for(;;); 
  }

  // 2. Clear the internal buffer 
  // (The library usually starts with an Adafruit logo in memory)
  display.clearDisplay();

  // 3. Optional: Initial visual feedback
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(F("System Ready..."));
  display.display();

  Serial.begin(9600);
}


void loop() {
   unsigned int sensorVal = readSensors(isInverse);
  
  // Toggle inverse mode when junction is hit
  if(detectJunction(sensorVal) == 1) {
    isInverse = !isInverse;
  }

  calculatePID(sensorVal);

  float correction = Kp * error + Kd * derivative;
  int leftMotor  = constrain(baseSpeed + correction, -255, 255);
  int rightMotor = constrain(baseSpeed - correction, -255, 255);

  // Update motor speed (uncomment when ready to drive)
  setMotorSpeed(rightMotor, leftMotor);
  // Update Display
  if (isInverse) {
    displayJunction(sensorVal, "INVERSE");
  } else {
    displayJunction(sensorVal, "NORMAL");
  }
}
