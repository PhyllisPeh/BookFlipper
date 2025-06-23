#include <Arduino.h>
#include <Servo.h>

// Pin definitions for DC Motors (L293D connections)
#define RIGHT_MOTOR_EN 6   // Enable pin for right motor
#define RIGHT_MOTOR_IN1 5  // Input 1 for right motor
#define RIGHT_MOTOR_IN2 4  // Input 2 for right motor

// Pin definitions for Servo Motors
#define LEFT_SERVO_PIN 10   // Left page holder servo
#define FLIP_SERVO_PIN 12   // Bottom page flipper servo

// Pin definitions for Ultrasonic Sensors
#define RIGHT_TRIG_PIN 13
#define RIGHT_ECHO_PIN A0

// Constants
#define DETECTION_DISTANCE 15  // Distance in cm to trigger page flip
#define MOTOR_SPEED 255       // DC motor speed (0-255)
#define SERVO_DELAY 1000      // Delay for servo movements
#define MOTOR_DELAY 2000      // Delay for DC motor movements
#define SERVO_UP_DELAY 500    // Delay for servo to move up

// Servo positions
#define SERVO_UP 0
#define SERVO_DOWN 90
#define FLIP_RIGHT 180
#define FLIP_LEFT 0

// Create servo objects
Servo leftServo;
Servo flipServo;

// Function declarations
void setupMotors();
void moveMotor(int enable, int in1, int in2, bool forward);
void stopMotor(int enable);
float getDistance(int trigPin, int echoPin);
bool performPageFlipForward();
void errorLog(const char* message);
void successLog(const char* message);

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing Book Flipper...");
  
  // Initialize motors and sensors
  setupMotors();
  
  // Attach servos
  leftServo.attach(LEFT_SERVO_PIN);
  flipServo.attach(FLIP_SERVO_PIN);
  
  // Setup ultrasonic sensor pins
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);
  
  // Initial position
  leftServo.write(SERVO_DOWN);
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
}

void setupMotors() {
  // Setup DC motor pins
  pinMode(RIGHT_MOTOR_EN, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  
  // Ensure motors are stopped
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
  delay(SERVO_UP_DELAY);

  // Step 3: Left servo movement
  leftServo.write(SERVO_UP);
  delay(SERVO_UP_DELAY);
  leftServo.write(SERVO_DOWN);
  delay(SERVO_DELAY);

  // Step 4: Return flip servo
  flipServo.write(FLIP_RIGHT);
  delay(SERVO_DELAY);
  
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
