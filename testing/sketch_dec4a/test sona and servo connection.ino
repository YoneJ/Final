
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
HCSR04 frontSonar(51, 50);

float previousTime_sona = 0.0;
float sampling_rate_sona = 500; //don vi la miliseconds

const int obstacleThreshold = 10; // cm
const int frontTrigPin = 51;
const int frontEchoPin = 50;

void setup() {
  servo.attach(6); // servo pin 6 
  Serial.begin(9600);
  pinMode(frontTrigPin, OUTPUT);
  pinMode(frontEchoPin, INPUT);
}
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
    servo.write(180);              
    delay(5000);                       // Waits 15ms for the servo to reach the position
    }
  else {
    servo.write(90);                  // Reset servo to position 0
    delay(5000);
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
