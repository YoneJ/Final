#include <HCSR04.h>
#include <Servo.h>
Servo servo;
HCSR04 frontSonar(51, 50);

float previousTime_sona = 0.0;
float sampling_rate_sona = 500;

const int obstacleThreshold = 4; 
const int frontTrigPin = 51;
const int frontEchoPin = 50;

void setup() {
  servo.attach(6);
  Serial.begin(9600);
  pinMode(frontTrigPin, OUTPUT);
  pinMode(frontEchoPin, INPUT);
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
  long distanceFront = get_front_distance();
  Serial.println(distanceFront);

  if (distanceFront == 200) { //Dm edge case
    servo.write(180);
    Serial.println("Obstacle detected at distance 0, servo moved to 180°");
  }
  else if (distanceFront < obstacleThreshold) {
    servo.write(180);
    Serial.println("Obstacle detected, servo moved to 180°");
  }
  else {
    servo.write(90);
    Serial.println("No obstacle detected, servo moved to 90°");
  }

  delay(500);
}
