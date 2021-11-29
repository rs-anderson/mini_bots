#include <Servo.h>
#include <SparkFun_TB6612.h>
#include <TimeLib.h>
#include <HCSR04.h>

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

// the different block-retrieval states we could be in
enum
{
  drivingToMiddleBlock,
  locatingBlock,
  approachingBlock,
  orientOnLine,
  findLine,
  driveOnLine,
  getClosestBlock,
  goToEnemyBase
};
unsigned char robotState = findLine;

class LineDetector
{

private:
  byte leftIrPin;
  byte rightIrPin;

public:
  LineDetector(byte leftIrPin, byte rightIrPin)
  {
    this->leftIrPin = leftIrPin;
    this->rightIrPin = rightIrPin;
    init();
  }
  void init()
  {
    pinMode(leftIrPin, INPUT);
    pinMode(rightIrPin, INPUT);
  }
  byte _getLeftIR()
  {
    return digitalRead(leftIrPin);
  }
  byte _getRightIR()
  {
    return digitalRead(rightIrPin);
  }

  byte getState()
  {
    if (_getLeftIR() == 1 && _getRightIR() == 0)
    {
      Serial.println("Line on left of robot (Left: 1, Right: 0)");
      return 0;
    }
    else if (_getLeftIR() == 0 && _getRightIR() == 1)
    {
      Serial.println("Line on right of robot (Left: 0, Right: 1)");
      return 1;
    }
    else if (_getLeftIR() == 0 && _getRightIR() == 0)
    {
      Serial.println("Robot aligned (Left: 0, Right: 0)");
      return 2;
    }
    else if (_getLeftIR() == 1 && _getRightIR() == 1)
    {
      Serial.println("Back at base (Left: 1, Right: 1)");
      return 3;
    }
  }
};

void reverseToLine()
{
  int lineState = getState();
  while (lineState == 3)
  {
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
}

void orientateOnLine()
{
  int lineState = getState();
  motor2.drive(100);
  motor1.drive(-100);
  delay(200);
  while (lineState != 0 && lineState != 1)
  {
    motor2.drive(100);
    motor1.drive(-100);
    //    left(motor1, motor2, 150);
    lineState = getState();
  }
  brake(motor1, motor2);
  delay(1000);
}

byte getState()
{
  int leftIRReading = digitalRead(leftIR);
  int rightIRReading = digitalRead(rightIR);
  if (leftIRReading == 1 && rightIRReading == 0)
  {
    Serial.println("Line on right of robot (Left: 1, Right: 0)");
    return 0;
  }
  else if (leftIRReading == 0 && rightIRReading == 1)
  {
    Serial.println("Line on left of robot (Left: 0, Right: 1)");
    return 1;
  }
  else if (leftIRReading == 0 && rightIRReading == 0)
  {
    Serial.println("Robot aligned (Left: 0, Right: 0)");
    return 2;
  }
  else if (leftIRReading == 1 && rightIRReading == 1)
  {
    Serial.println("Back at base (Left: 1, Right: 1)");
    return 3;
  }
}

int driveAlongLine()
{
  int lineState = getState();
  Serial.println(lineState);
  while (lineState == 3)
  {
    motor1.drive(110);
    motor2.drive(110);
    lineState = getState();
    if (lineState == 1)
    {
      brake(motor1, motor2);
      motor2.drive(110);
      motor1.drive(0);
      lineState = getState();
    }
    else if (lineState == 0)
    {
      brake(motor1, motor2);
      motor2.drive(0);
      motor1.drive(110);
      lineState = getState();
    }
  }
}

void setup()
{
  Serial.begin(9600);
  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);
}

void loop()
{

  driveAlongLine();
  //
  //    switch (robotState) {
  //
  //      case findLine:
  //         reverseToLine();
  //         robotState = orientOnLine;
  //
  //      case orientOnLine:
  //        Serial.println("Going home along line...");
  //        orientateOnLine();  // align robot on the line
  //        robotState = driveOnLine;
  //
  //      case driveOnLine:
  //          robotState = driveAlongLine();  // move along line in the direction of home
  ////        navigateAroundBlock();  // move around closest block (it is blocking path home)
  ////        driveAlongLine(200, 10);
  //
  //
  //      default:
  //          break;
  //   }
}
