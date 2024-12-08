#include <HCSR04.h>
#include <Servo.h>
Servo servo;
HCSR04 frontSonar(51, 50);

float previousTime_sona = 0.0;
float sampling_rate_sona = 500;

const int obstacleThreshold = 3; 
const int frontTrigPin = 51;
const int frontEchoPin = 50;

int enL = 7; // enable the left motor driver, using PWM  
int inL1 = 8; // control direction of the left motor 
int inL2 = 9;
int enR = 13; // enable the right motor driver, using PWM  
int inR1 = 10; // control direction of the right motor 
int inR2 = 11;

// Initial motor speed
int initial_motor_speed = 80;
int adjusted_speed = initial_motor_speed;

// PID variables
float PID_value = 0;  // PID correction value from Python
int left_motor_speed = 0, right_motor_speed = 0;

void stop() {
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
}

void setup() {
  servo.attach(6);
  Serial.begin(9600);
  pinMode(frontTrigPin, OUTPUT);
  pinMode(frontEchoPin, INPUT);

  pinMode(enL, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);
  
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);
}

float get_front_distance() {
  float distance = frontSonar.dist();
  if (distance == 0.0) {
    distance = 200.0;
  }
  else if (distance <= 5.0) {
    distance == 5.0;
  }
  return distance;
}


void loop() {
  if (Serial.available() >= sizeof(float)) {
  Serial.readBytes((char*)&PID_value, sizeof(PID_value));

  if (PID_value == -1.0) {
    stop();
    Serial.println("stop");
  } else {
    Serial.print("PID value received: ");
    Serial.println(PID_value);

  long distanceFront = get_front_distance();
  Serial.print("Front Distance: ");
  Serial.println(distanceFront);

    int adjusted_speed = initial_motor_speed;
    if (distanceFront < 10) {
      adjusted_speed = map(distanceFront, 5, 10, 40, initial_motor_speed);
      adjusted_speed = max(adjusted_speed, 30);
    } else if (distanceFront < 20) {
      adjusted_speed = map(distanceFront, 10, 20, 60, initial_motor_speed);
    }

    left_motor_speed = adjusted_speed + PID_value;
    right_motor_speed = adjusted_speed - PID_value;

    left_motor_speed = constrain(left_motor_speed, 0, 150);
    right_motor_speed = constrain(right_motor_speed, 0, 150);

    if (left_motor_speed > 0) {
      digitalWrite(inL1, HIGH);
      digitalWrite(inL2, LOW);
    } else {
      digitalWrite(inL1, LOW);
      digitalWrite(inL2, HIGH);
      left_motor_speed = -left_motor_speed; 
    }

    if (right_motor_speed > 0) {
      digitalWrite(inR1, LOW);
      digitalWrite(inR2, HIGH);
    } else {
      digitalWrite(inR1, HIGH);
      digitalWrite(inR2, LOW);
      right_motor_speed = -right_motor_speed;
    }

    analogWrite(enL, left_motor_speed);
    analogWrite(enR, right_motor_speed);
    Serial.print("left: ");
    Serial.println(left_motor_speed);
    Serial.print("right: ");
    Serial.println(right_motor_speed);

  if (distanceFront < obstacleThreshold) {
    servo.write(180);
    Serial.println("Obstacle detected, servo moved to 180°");
    Serial.println("Grab Done");

  }
  else {
    servo.write(90);
    Serial.println("No obstacle detected, servo moved to 90°");
  }

  delay(500);
    }
  }
}
