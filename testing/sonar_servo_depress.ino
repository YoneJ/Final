#include <HCSR04.h>
#include <Servo.h>

// Motor control pins
int enL = 7; // enable the left motor driver, using PWM  
int inL1 = 8; // control direction of the left motor 
int inL2 = 9;
int enR = 13; // enable the right motor driver, using PWM  
int inR1 = 10; // control direction of the right motor 
int inR2 = 11;

// Initial motor speed
int initial_motor_speed = 60;

// PID variables
float PID_value = 0;  // PID correction value from Python
int left_motor_speed = 0, right_motor_speed = 0;

// Sonar and Servo setup
Servo servo;
HCSR04 frontSonar(53, 52);

float previousTime_sona = 0.0;
float sampling_rate_sona = 500; // In milliseconds
const int obstacleThreshold = 10; // cm
const int frontTrigPin = 53;
const int frontEchoPin = 52;
bool bottleGrabbed = false;

void setup() {
  Serial.begin(9600);  // Start serial communication at 9600 baud rate
  pinMode(enL, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);
  
  // Initialize motor direction (forward)
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);

  // Initialize servo
  servo.attach(6); // Servo pin 6
}

void stop() {
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
}

void rotate() {
  analogWrite(enL, initial_motor_speed);
  analogWrite(enR, initial_motor_speed);
  Serial.println("rotate");
}

float get_front_distance() {
  float distance = frontSonar.dist();
  if (distance == 0.0) {
    distance = 200.0;
  }
  return distance;
}

void grab_bottle() {
  for (int pos = 0; pos <= 180; pos += 1) {
    servo.write(pos);
    delay(15);
  }
  bottleGrabbed = true;
  Serial.println("Bottle grabbed.");
}

void loop() {
  if (Serial.available() >= sizeof(char) + sizeof(float)) {
    char command;
    float value;
    Serial.readBytes(&command, sizeof(char));  // Read the command
    Serial.readBytes((char*)&value, sizeof(float));  // Read the float value

    if (command == 'S') {
      stop();
      Serial.println("stop");
    } else if (command == 'R') {
      rotate();
    } else if (command == 'P') {
      PID_value = value;

      // Debugging: Print the PID value received from Python
      Serial.print("PID value received: ");
      Serial.println(PID_value);

      // Calculate motor speed based on the PID output
      left_motor_speed = initial_motor_speed + PID_value;
      right_motor_speed = initial_motor_speed - PID_value;

      // Constrain motor speeds to a safe range 
      left_motor_speed = constrain(left_motor_speed, 0, 80);
      right_motor_speed = constrain(right_motor_speed, 0, 80);

      // Control left motor
      if (left_motor_speed > 0) {
        digitalWrite(inL1, HIGH);
        digitalWrite(inL2, LOW);
      } else {
        digitalWrite(inL1, LOW);
        digitalWrite(inL2, HIGH);
        left_motor_speed = -left_motor_speed;  // Make the speed positive for PWM
      }

      // Control right motor
      if (right_motor_speed > 0) {
        digitalWrite(inR1, LOW);
        digitalWrite(inR2, HIGH);
      } else {
        digitalWrite(inR1, HIGH);
        digitalWrite(inR2, LOW);
        right_motor_speed = -right_motor_speed;  // Make the speed positive for PWM
      }

      // Set motor speeds using PWM
      analogWrite(enL, left_motor_speed);  // Set left motor speed
      analogWrite(enR, right_motor_speed); // Set right motor speed
      Serial.print("left: ");
      Serial.println(left_motor_speed);
      Serial.print("right: ");
      Serial.println(right_motor_speed);
    }
  }

  // Sonar and Servo Control
  float distanceFront = get_front_distance();
  if (distanceFront < obstacleThreshold && !bottleGrabbed) {
    grab_bottle();
  }
}
