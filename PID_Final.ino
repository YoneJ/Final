// Motor control pins
int enL = 7; // enable the left motor driver, using PWM  
int inL1 = 8; // control direction of the left motor 
int inL2 = 9;
int enR = 13; // enable the right motor driver, using PWM  
int inR1 = 10; // control direction of the right motor 
int inR2 = 11;

// For Encoder
int enLA = 2;  // left motor encoder
int enLB = 3;
int enRA = 18; // right motor encoder 
int enRB = 19;

// PID variables
float Kp = 4.5, Ki = 0.02, Kd = 0.5;
float sampling_rate = 100; // Adjusted sampling rate
float previousTime;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0;

// Initial motor speed
int initial_motor_speed = 90;

void setup() {
  Serial.begin(9600);
  previousTime = 0; 
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

void loop() {
  if (Serial.available()) {
    error = Serial.parseFloat();
    Serial.println(error); // Send the error back to Raspberry Pi
    calculate_pid(error);
    motor_control();
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
  left_motor_speed = constrain(left_motor_speed, 0, 150);
  right_motor_speed = constrain(right_motor_speed, 0, 150);

  analogWrite(enL, left_motor_speed);  // Set left motor speed
  analogWrite(enR, right_motor_speed); // Set right motor speed
}
