#include <Servo.h>
#include <SparkFun_TB6612.h>
#include <TimeLib.h>
#include <HCSR04.h>


//////////////////////////////////
// WRAPPER CLASSES FOR SENSORS ///
//////////////////////////////////


//class Gripper {
//
//  private:
//    Servo servo;
//    byte pin;
//    byte initAngle;
//
//public:
//  Gripper(byte pin, byte initAngle)
//  {
//    this->pin = pin;
//    this->initAngle = initAngle;
//    init();
//  }
//  void init()
//  {
//    servo.attach(pin);
//    open();
//  }
//  byte getAngle(byte desiredAngle)
//  {
//    byte currentAngle = initAngle;
//    if (currentAngle < desiredAngle)
//    {
//      while (currentAngle <= desiredAngle)
//      {
//        servo.write(currentAngle);
//        currentAngle++;
//        delay(10);
//      }
//    }
//    else
//    {
//      while (currentAngle >= desiredAngle)
//      {
//        servo.write(currentAngle);
//        currentAngle--;
//        delay(10);
//      }
//    }
//
//    return currentAngle;
//  }
//  void dropBlock()
//  {
//    servo.write(initAngle);
//  }
//  void open()
//  {
//    servo.write(initAngle);
//  }
//  void pickUpBlock(byte desiredAngle)
//  {
////    byte currentAngle = getAngle(desiredAngle);
//    servo.write(20);
//  }
//};


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


//////////////////////////////////////
// INITIALISING OBJECTS & VARIABLES //
//////////////////////////////////////

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
// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguments you can either write new functions or
// call the function more than once.
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

#define leftIR A0
#define rightIR A1
//LineDetector lineDetector(leftIR, rightIR);

//byte pin = 2;
//byte initialAngle = 80;
//Gripper gripper(pin, initialAngle);

int triggerPin = 10;
int echoPin = 11;
UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);

long time;
long timeElapsed;
double distanceToBlock;
bool inPosition;


// the different block-retrieval states we could be in
enum {drivingToMiddleBlock, locatingBlock, approachingBlock, orientOnLine, findLine, driveOnLine, getClosestBlock, goToEnemyBase};
unsigned char robotState = findLine; 


////////////////////////
// HELPFUL FUNCTIONS //
///////////////////////


void _swap(double *p, double *q) {
   int t;
   
   t=*p; 
   *p=*q; 
   *q=t;
}


void _sort(double a[], int n) { 
   int i, j;
 
   for(i=0;i<n-1;i++) {
      for(j=0;j<n-i-1;j++) {
         if(a[j]>a[j+1])
            _swap(&a[j],&a[j+1]);
      }
   }
}


double getMedianDistance(){
  // denoising the data by taking median of three measurements
  int n = 5;

  double distance_1 = distanceSensor.measureDistanceCm();
  double distance_2 = distanceSensor.measureDistanceCm();
  double distance_3 = distanceSensor.measureDistanceCm();
  double distance_4 = distanceSensor.measureDistanceCm();
  double distance_5 = distanceSensor.measureDistanceCm();
  double distances[] = {distance_1, distance_2, distance_3, distance_4, distance_5};

  _sort(distances, n);
  n = (n+1) / 2 - 1;

  double median_distance = distances[n];
  return median_distance;
}

void driveToLocation(long duration, int speed){

    Serial.println("Driving to location...");
    time = millis();
    timeElapsed = 0;

    while (timeElapsed < duration){
      motor1.drive(speed);
      motor2.drive(speed + 2.5);
     forward(motor1, motor2, speed);
      timeElapsed = millis() - time;
    }

    brake(motor1, motor2);
    Serial.println("Arrvived at location...");
    delay(1000);
}


double _scanForBlockInDirection(long duration, int direction){
    // direction == 1 -> left; direction == -1 -> right

    double distance;
    time = millis();
    timeElapsed = 0;

    while (timeElapsed < duration){
      if (direction == 1) left(motor1, motor2, 175);
      else right(motor1, motor2, 200);
       
       delay(200);
       brake(motor1, motor2);
       delay(200);
        distance = getMedianDistance();

        if (distance < 30 && distance > 0){
          if (direction == 1) left(motor1, motor2, 175);
          else right(motor1, motor2, 200);
          delay(200);
          brake(motor1, motor2);
          Serial.println("Detected object at distance:");
          Serial.println(distance);
          delay(1000);
          return distance;
        }
        timeElapsed = millis() - time;

    }
    brake(motor1, motor2);
    return -1;
}


double scanForBlock(long duration){

    double distance;

    Serial.println("Scanning in left direction...");
    distance = _scanForBlockInDirection(duration, 1); // scanning left

    if (distance > -1){
      return distance; // found block on left rotation
    }

    Serial.println("No block found on left rotation. Scanning in right direction...");
    distance = _scanForBlockInDirection(duration*2, -1);

    if (distance > -1){
      return distance;  // found block on right rotation
    }

    Serial.println("No block found on right rotation either...");
    return distance;  // did not find block
}


bool approachBlock(double originalDistance){

  int speed = 110;
  int distForGrippers = 6;
  double distance = originalDistance;
  int retries = 3;
  
  Serial.println("Approaching block...");
  while (distance > distForGrippers){
    
//    forward(motor1, motor2, speed);
    motor1.drive(speed);
    motor2.drive(speed - 10);
//    brake(motor1, motor2);
    distance = getMedianDistance();
    
    Serial.println("Distance to block...");
    Serial.println(distance);
//    delay(1000);

    if (distance > originalDistance + 10 || distance == -1){
      Serial.println("We've lost the block!!!");
      brake(motor1, motor2);
      delay(1000);
      Serial.println("Retrying three times...");
      for (int i = 0; i < retries; i++){
        distance = getMedianDistance();
        if (distance > originalDistance + 10 || distance == -1) continue;
        else break;
      }
      if (distance > originalDistance + 10 || distance == -1){
        Serial.println("Couldn't locate block again, beginning search...");
        driveToLocation(200, -150);  // backtrack a little
        distance = scanForBlock(2000); // search for the block again
        if (distance == -1) return false;
        continue;
      }
      Serial.println("Found the lost block!");
    }
  }
  brake(motor1, motor2);
//  delay(1000);
  return true;
}



void reverseToLine(){
  int lineState = getState();
  while (lineState == 3){
  forward(motor1, motor2, -110);
  lineState = getState();
//      Serial.println(lineState);
  }
  brake(motor1, motor2);
  delay(1000);
  forward(motor1, motor2, -110);
  delay(200);
  brake(motor1, motor2);
  delay(1000);
//  forward(motor1, motor2, -110);
//  delay(500);
//  brake(motor1, motor2);
}

void orientateOnLine(){
  int lineState = getState();
  motor2.drive(100);
  motor1.drive(-100);
  delay(200);
  while (lineState != 0 && lineState != 1){
    motor2.drive(100);
    motor1.drive(-100);
//    left(motor1, motor2, 150);
    lineState = getState();
  }
  brake(motor1, motor2);
  delay(1000);
}


int driveAlongLine(){

  while (true){
   int lineState = getState();
  
  if (lineState == 0){
    motor1.drive(100);
    motor2.drive(110);
  }
  else if (lineState == 1){
    motor1.drive(110);
    motor2.drive(100);
  }

  else if (lineState == 2){
    motor1.drive(110);
    motor2.drive(110);
  }

  else if (lineState == 3) {
    return findLine;
  }
  }

  return driveOnLine;

}

void navigateAroundBlock(){
  ;
}


bool isObstacle(){
  double distance = getMedianDistance();
  if (distance < 15) return true;
  return false;
}


Servo servo;
void pickUpBlock(){
    time = millis();
    timeElapsed = 0;
    int servoAngle = 20;
    while (servoAngle < 91){
      forward(motor1, motor2, 115);
      servo.write(servoAngle);
      delay(300);
      servoAngle = servoAngle + 10;
      Serial.println(servoAngle);
    }
    brake(motor1, motor2);
    delay(300);
}

byte getState(){
    int leftIRReading = digitalRead(leftIR);
    int rightIRReading = digitalRead(rightIR);
    if (leftIRReading == 1 && rightIRReading == 0){
      Serial.println("Line on right of robot (Left: 1, Right: 0)");
      return 0;
    }
    else if (leftIRReading == 0 && rightIRReading == 1){
      Serial.println("Line on left of robot (Left: 0, Right: 1)");
      return 1;
    }
    else if (leftIRReading == 0 && rightIRReading == 0){
      Serial.println("Robot aligned (Left: 0, Right: 0)");
      return 2;
    }
    else if (leftIRReading == 1 && rightIRReading == 1){
      Serial.println("Back at base (Left: 1, Right: 1)");
      return 3;
    }
  }


//////////
// MAIN //
//////////


void setup()
{
  Serial.begin(9600);
  servo.attach(2);
  servo.write(20);
  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);
}


void loop(){

    switch (robotState) {

      case drivingToMiddleBlock:
        Serial.println("Driving to middle block...");
        driveToLocation(2500, 250);  // move to location near middle block
        robotState = locatingBlock;
        break;


      case locatingBlock:
        Serial.println("Locating block...");
        distanceToBlock = scanForBlock(2000);  // determine distance to block and align robot
        if (distanceToBlock > 0){
           robotState = approachingBlock;
           delay(1000);
        }
        else{
          Serial.println("No block located. Trying again.");
          delay(1000);   // what to do when we don't find block after scanning left and right?
        }
        break;

    
      case approachingBlock:
        Serial.println("Approaching block...");
        inPosition = approachBlock(distanceToBlock);  // get into position to pick up block
        if (inPosition) pickUpBlock();  // use grippers to pick up block
        robotState = findLine;  // take the block home
//        what to do if we lose the block? Maybe just go home on line and go for closer block.

      case findLine:
         reverseToLine();
         robotState = orientOnLine;
      
      case orientOnLine:
        Serial.println("Going home along line...");
        orientateOnLine();  // align robot on the line
        robotState = driveOnLine;

      case driveOnLine:
          robotState = driveAlongLine();  // move along line in the direction of home
//        navigateAroundBlock();  // move around closest block (it is blocking path home)
//        driveAlongLine(200, 10);


      default:
          break;
   }
}

