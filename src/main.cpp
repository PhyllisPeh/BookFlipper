#include <Arduino.h>
#include <Servo.h>

// Pin definitions for DC Motors (L293D connections)
#define LEFT_MOTOR_EN 9    // Enable pin for left motor
#define LEFT_MOTOR_IN1 8   // Input 1 for left motor
#define LEFT_MOTOR_IN2 7   // Input 2 for left motor
#define RIGHT_MOTOR_EN 6   // Enable pin for right motor
#define RIGHT_MOTOR_IN1 5  // Input 1 for right motor
#define RIGHT_MOTOR_IN2 4  // Input 2 for right motor

// Pin definitions for Servo Motors
#define LEFT_SERVO_PIN 10   // Left page holder servo
#define RIGHT_SERVO_PIN 11  // Right page holder servo
#define FLIP_SERVO_PIN 12   // Bottom page flipper servo

// Pin definitions for Ultrasonic Sensors
#define LEFT_TRIG_PIN 2
#define LEFT_ECHO_PIN 3
#define RIGHT_TRIG_PIN 13
#define RIGHT_ECHO_PIN A0

// Constants
#define DETECTION_DISTANCE 15  // Distance in cm to trigger page flip
#define MOTOR_SPEED 200       // DC motor speed (0-255)
#define SERVO_DELAY 1000      // Delay for servo movements
#define MOTOR_DELAY 1500      // Delay for DC motor movements

// Servo positions
#define SERVO_UP 0
#define SERVO_DOWN 90
#define FLIP_RIGHT 0
#define FLIP_LEFT 180

// Create servo objects
Servo leftServo;
Servo rightServo;
Servo flipServo;

// Function declarations
void setupMotors();
void moveMotor(int enable, int in1, int in2, bool forward);
void stopMotor(int enable);
float getDistance(int trigPin, int echoPin);
bool performPageFlipForward();
bool performPageFlipBackward();
void errorLog(const char* message);
void successLog(const char* message);

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing Book Flipper...");
  
  // Initialize motors and sensors
  setupMotors();
  
  // Attach servos
  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN);
  flipServo.attach(FLIP_SERVO_PIN);
  
  // Setup ultrasonic sensor pins
  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);
  
  // Initial position
  leftServo.write(SERVO_DOWN);
  rightServo.write(SERVO_DOWN);
  flipServo.write(FLIP_RIGHT);
  
  delay(1000);
  successLog("Initialization complete");
}

void loop() {
  // Check right sensor for forward flip
  float rightDistance = getDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
  if (rightDistance > 0 && rightDistance < DETECTION_DISTANCE) {
    Serial.println("Right motion detected - Starting forward page flip");
    if (performPageFlipForward()) {
      successLog("Forward page flip completed");
    }
    delay(2000); // Prevent multiple triggers
  }
  
  // Check left sensor for backward flip
  float leftDistance = getDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
  if (leftDistance > 0 && leftDistance < DETECTION_DISTANCE) {
    Serial.println("Left motion detected - Starting backward page flip");
    if (performPageFlipBackward()) {
      successLog("Backward page flip completed");
    }
    delay(2000); // Prevent multiple triggers
  }
}

void setupMotors() {
  // Setup DC motor pins
  pinMode(LEFT_MOTOR_EN, OUTPUT);
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_EN, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  
  // Ensure motors are stopped
  stopMotor(LEFT_MOTOR_EN);
  stopMotor(RIGHT_MOTOR_EN);
}

void moveMotor(int enable, int in1, int in2, bool forward) {
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  analogWrite(enable, MOTOR_SPEED);
}

void stopMotor(int enable) {
  analogWrite(enable, 0);
}

float getDistance(int trigPin, int echoPin) {
  // Generate ultrasonic pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Measure the response
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2; // Convert to cm
}

bool performPageFlipForward() {
  // Step 1: Move right DC motor forward
  moveMotor(RIGHT_MOTOR_EN, RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, true);
  delay(MOTOR_DELAY);
  stopMotor(RIGHT_MOTOR_EN);
  
  // Step 2: Flip the page
  flipServo.write(FLIP_LEFT);
  delay(SERVO_DELAY);
  
  // Step 3: Left servo movement
  leftServo.write(SERVO_UP);
  delay(SERVO_DELAY);
  leftServo.write(SERVO_DOWN);
  delay(SERVO_DELAY);
  
  // Step 4: Return flip servo
  flipServo.write(FLIP_RIGHT);
  delay(SERVO_DELAY);
  
  // Step 5: Return right DC motor
  moveMotor(RIGHT_MOTOR_EN, RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, false);
  delay(MOTOR_DELAY);
  stopMotor(RIGHT_MOTOR_EN);
  
  return true;
}

bool performPageFlipBackward() {
  // Step 1: Move flip servo left
  flipServo.write(FLIP_LEFT);
  delay(SERVO_DELAY);
  
  // Step 2: Move left DC motor forward
  moveMotor(LEFT_MOTOR_EN, LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, true);
  delay(MOTOR_DELAY);
  stopMotor(LEFT_MOTOR_EN);
  
  // Step 3: Flip the page
  flipServo.write(FLIP_RIGHT);
  delay(SERVO_DELAY);
  
  // Step 4: Right servo movement
  rightServo.write(SERVO_UP);
  delay(SERVO_DELAY);
  rightServo.write(SERVO_DOWN);
  delay(SERVO_DELAY);
  
  // Step 5: Return left DC motor
  moveMotor(LEFT_MOTOR_EN, LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, false);
  delay(MOTOR_DELAY);
  stopMotor(LEFT_MOTOR_EN);
  
  return true;
}

void errorLog(const char* message) {
  Serial.print("ERROR: ");
  Serial.println(message);
}

void successLog(const char* message) {
  Serial.print("SUCCESS: ");
  Serial.println(message);
}