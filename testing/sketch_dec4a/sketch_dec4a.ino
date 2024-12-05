// Motor control pins
int enL = 7; // enable the left motor driver, using PWM  
int inL1 = 8; // control direction of the left motor 
int inL2 = 9;
int enR = 13; // enable the right motor driver, using PWM  
int inR1 = 10; // control direction of the right motor 
int inR2 = 11;

// PID variables
float Kp = 1.5, Ki = 0.0, Kd = 0.1;  // Adjusted PID constants
float previousTime;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0;

// Initial motor speed
int initial_motor_speed = 55;  // Increased initial motor speed

void setup() {
  servo.attach(6); // servo pin 9 
  Serial.begin(9600);
  previousTime = 0; 
  pinMode(enL, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);

  servo.attach(6); // servo pin 6 
  Serial.begin(9600);
  pinMode(frontTrigPin, OUTPUT);
  pinMode(frontEchoPin, INPUT);
  
  // Initialize motor direction (forward)
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);
}

void loop() {
  if (Serial.available()) {
    String message = Serial.readStringUntil('\n');
    if (message == "stop") {
      stop();
      Serial.println("Stopped");
    } else {
      error = message.toFloat();
      Serial.println(error);  // Print the error for debugging
      calculate_pid(error);
      motor_control();
    }
  }
}

void stop() {
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
}

void calculate_pid(float error) {
  P = error;
  I += error;
  D = error - previous_error;
  
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previous_error = error;
}

void motor_control() {
  // Calculate the effective motor speed based on PID
  int left_motor_speed = initial_motor_speed + PID_value;
  int right_motor_speed = initial_motor_speed - PID_value;

  // Constrain motor speeds 
  left_motor_speed = constrain(left_motor_speed, 0, 80);
  right_motor_speed = constrain(right_motor_speed, 10, 80);

  // Control left motor
  if (left_motor_speed > 0) {
    digitalWrite(inL1, HIGH);
    digitalWrite(inL2, LOW);
  } else if (left_motor_speed < 0) {
    digitalWrite(inL1, LOW);
    digitalWrite(inL2, HIGH);
    left_motor_speed = -left_motor_speed;  // Make the speed positive for PWM
  } else {
    digitalWrite(inL1, LOW);
    digitalWrite(inL2, LOW);
  }

  // Control right motor
  if (right_motor_speed > 0) {
    digitalWrite(inR1, HIGH);
    digitalWrite(inR2, LOW);
  } else if (right_motor_speed < 0) {
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, HIGH);
    right_motor_speed = -right_motor_speed;  // Make the speed positive for PWM
  } else {
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, LOW);
  }

  // Set motor speeds using PWM
  analogWrite(enL, left_motor_speed);  // Set left motor speed
  analogWrite(enR, right_motor_speed); // Set right motor speed
}

//        .==.        .==.
//       //`^\\      //^`\\
//      // ^ ^\(\__/)/^ ^^\\
//     //^ ^^ ^/6  6\ ^^ ^ \\
//    //^ ^^ ^/( .. )\^ ^ ^ \\
//   // ^^ ^/\| v""v |/\^ ^ ^\\
//  // ^^/\/ /  `~~`  \ \/\^ ^\\
//  -----------------------------
/// HERE BE DRAGONS TO MARK


// Code sonar + servo 
#include <HCSR04.h>
#include <Servo.h>
Servo servo;
// Ultrasonic sensor pins
// Ý tưởng: cứ có vật thể trước mặt trong khoảng 10cm thì quay servo 180 độ để grab vật thể
// Câu hỏi: làm sao để giữ nguyên trạng thái của servo? (điều kiện hiện tại là vật thể cách 10cm thì xoay)
// Khi nào thì thả ra nữa? 
HCSR04 frontSonar(53, 52);

float previousTime_sona = 0.0;
float sampling_rate_sona = 500; //don vi la miliseconds

const int obstacleThreshold = 10; // cm
const int frontTrigPin = 53;
const int frontEchoPin = 52;

int pos = 0;    // variable to store the servo position

float get_front_distance() {
  float distance = frontSonar.dist();
  if (distance == 0.0) {
    distance = 200.0;
  }
  return distance;
}



void loop() {
  long distanceFront = get_front_distance();
  Serial.println(distanceFront);
  if (distanceFront < obstacleThreshold) {
    for (int pos = 0; pos <= 180; pos += 1) { // Goes from 0 degrees to 180 degrees
      servo.write(pos);              
      delay(15);                       // Waits 15ms for the servo to reach the position
    }
  } else {
      servo.write(0);                  // Reset servo to position 0
  }
}
//        .==.        .==.
//       //`^\\      //^`\\
//      // ^ ^\(\__/)/^ ^^\\
//     //^ ^^ ^/6  6\ ^^ ^ \\
//    //^ ^^ ^/( .. )\^ ^ ^ \\
//   // ^^ ^/\| v""v |/\^ ^ ^\\
//  // ^^/\/ /  `~~`  \ \/\^ ^\\
//  -----------------------------
/// HERE BE DRAGONS
