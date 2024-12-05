

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
