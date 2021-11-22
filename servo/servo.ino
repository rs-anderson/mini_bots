////www.elegoo.com
////2018.12.19
//#include <Servo.h>
//Servo myservo;
//
//void setup(){
//  myservo.attach(12);
//  myservo.write(90);// move servos to center position -> 90°
//  
//} 
//void loop(){
//  myservo.write(45);// move servos to center position -> 90°
//  delay(1500);
//  myservo.write(90);
//  delay(1000);
//}

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(12);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  for (pos = 45; pos <= 90; pos += 0.5) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 90; pos >= 45; pos -= 0.5) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}
