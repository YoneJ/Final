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
}

void stop() {
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
}

void loop() {
  if (Serial.available() >= sizeof(float)) {
    Serial.readBytes((char*)&PID_value, sizeof(PID_value));  // Read the float value directly

    if (PID_value == -1.0) {
      stop();
      Serial.println("stop");
    } else {
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
}
