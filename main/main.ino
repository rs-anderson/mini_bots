#include <Servo.h>
#include <SparkFun_TB6612.h>
#include <TimeLib.h>
#include <HCSR04.h>


class Gripper {

  private:
    Servo servo;
    byte pin;
    byte initAngle;

  public:
    Gripper(byte pin, byte initAngle) {
      this->pin = pin;
      this->initAngle = initAngle;
      init();
    }
    void init() {
      servo.attach(pin);
      open();
    }
    void pickUpBlock() {
      // incomplete
      ;
    }
    void open() {
      servo.write(initAngle);
    }
};


class LineDetector {

  private:
    byte leftIrPin;
    byte rightIrPin;

  public:
    LineDetector(byte leftIrPin, byte rightIrPin) {
      this->leftIrPin = leftIrPin;
      this->rightIrPin = rightIrPin;
      init();
    }
    void init() {
      pinMode(leftIrPin, INPUT);
      pinMode(rightIrPin, INPUT);
    }
    byte _getLeftIR() {
      return digitalRead(leftIrPin);
    }
    byte _getRightIR() {
      return digitalRead(rightIrPin);
    }

    byte getState(){
      if (_getLeftIR() == 1 && _getRightIR() == 0){
        Serial.println("Line on left of robot (Left: 1, Right: 0)");
        return 0;
      }
      else if (_getLeftIR() == 0 && _getRightIR() == 1){
        Serial.println("Line on right of robot (Left: 0, Right: 1)");
        return 1;
      }
      else if (_getLeftIR() == 0 && _getRightIR() == 0){
        Serial.println("Robot aligned (Left: 0, Right: 0)");
        return 2;
      }
      else if (_getLeftIR() == 1 && _getRightIR() == 1){
        Serial.println("Back at base (Left: 1, Right: 1)");
        return 3;
      }
    }
};



#define leftIR A2
#define rightIR A3


// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
#define AIN1 3
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

int triggerPin = 10;
int echoPin = 11;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

LineDetector lineDetector(leftIR, rightIR);
Gripper gripper(5, 90);
UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);

long time;
long diff;
double distanceToBlock;

enum {getMiddleBlock, getHomeOnLine, getClosestBlock, goToEnemyBase};
unsigned char robotState = getMiddleBlock; 

void setup()
{
  Serial.begin(9600);
}


void swap(double *p, double *q) {
   int t;
   
   t=*p; 
   *p=*q; 
   *q=t;
}

void sort(double a[], int n) { 
   int i, j;
 
   for(i=0;i<n-1;i++) {
      for(j=0;j<n-i-1;j++) {
         if(a[j]>a[j+1])
            swap(&a[j],&a[j+1]);
      }
   }
}


double getMedianDistance(){
  int n = 3;
  double distance_1 = distanceSensor.measureDistanceCm();
  double distance_2 = distanceSensor.measureDistanceCm();
  double distance_3 = distanceSensor.measureDistanceCm();

  double distances[] = {distance_1, distance_2, distance_3};
  sort(distances, n);
  n = (n+1) / 2 - 1;
  double median_distance = distances[n];
  return median_distance;
}

void driveToLocation(){
    Serial.println("Driving to location...");
    time = millis();
    diff = 0;
    while (diff < 2500){
      forward(motor1, motor2, 250);
      diff = millis() - time;
    }
    brake(motor1, motor2);
    Serial.println("Arrvived at location...");
}


double scanLeftForBlock(long duration){
    Serial.println("Scanning left for block...");
    double distance;
    time = millis();
    diff = 0;
    while (diff < duration){  // turning for max 1s
        left(motor1, motor2, 200);
//        distance = distanceSensor.measureDistanceCm();
        distance = getMedianDistance();
        if (distance < 15 && distance > 0){
          brake(motor1, motor2);
          Serial.println("Detected object at distance:");
          Serial.println(distance);
          return distance;
        }
        diff = millis() - time;
    }
    return -1;
}

double scanRightForBlock(long duration){
    Serial.println("Scanning right for block...");
    double distance;
    time = millis();
    diff = 0;
    while (diff < duration){
        right(motor1, motor2, 200);
//        distance = distanceSensor.measureDistanceCm();
        distance = getMedianDistance();
        if (distance < 15 && distance > 0){
          brake(motor1, motor2);
          Serial.println("Detected object at distance:");
          Serial.println(distance);
          return distance;
        }
        diff = millis() - time;
    }
    return -1;
}


double scanForBlock(long duration){
    double distance;
    distance = scanLeftForBlock(duration);
    if (!distance == -1){
      return distance; // found block on left rotation
    }
    Serial.println("No block found on left rotation...");
    distance = scanRightForBlock(duration*2);
    if (!distance == -1){
      return distance;  // found block on right rotation
    }
    return distance;  // did not find block
}


void approachBlock(double originalDistance){
  int speed = 50;
  double distance = originalDistance;
  Serial.println("Approaching block...");
  while (distance > 2){
    forward(motor1, motor2, speed);
//    distance = distanceSensor.measureDistanceCm();
    distance = getMedianDistance();
    Serial.println("Distance to block...");
    Serial.println(distance);
    if (distance > originalDistance + 10){
      Serial.println("We've lost the block!!!");
      brake(motor1, motor2);
      break;
    }
  }
}

void loop(){
    
    switch (robotState) {

      case getMiddleBlock:
          Serial.println("Going to get middle block");
          driveToLocation();
          distanceToBlock = scanForBlock(2000);
          if (distanceToBlock > 0){
             approachBlock(distanceToBlock);
          }
          else{
            ;
          }
          break;

      default:
          break;
   }
}
