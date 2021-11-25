#include <Servo.h>
#include <SparkFun_TB6612.h>
#include <TimeLib.h>
#include <HCSR04.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
//////////////////////////////////
// WRAPPER CLASSES FOR SENSORS ///
//////////////////////////////////

class Gripper
{

private:
  Servo servo;
  byte pin;
  byte initAngle;

public:
  Gripper(byte pin, byte initAngle)
  {
    this->pin = pin;
    this->initAngle = initAngle;
    init();
  }
  void init()
  {
    servo.attach(pin);
    open();
  }
  byte getAngle(byte desiredAngle)
  {
    byte currentAngle = initAngle;
    if (currentAngle < desiredAngle)
    {
      while (currentAngle <= desiredAngle)
      {
        servo.write(currentAngle);
        currentAngle++;
        delay(10);
      }
    }
    else
    {
      while (currentAngle >= desiredAngle)
      {
        servo.write(currentAngle);
        currentAngle--;
        delay(10);
      }
    }

    return currentAngle;
  }
  void dropBlock()
  {
    servo.write(initAngle);
  }
  void open()
  {
    servo.write(initAngle);
  }
};

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

class Location
{

private:
  const int MPU_ADDR = 0x68;                                 // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
  int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
  int16_t gyro_x, gyro_y, gyro_z;                            // variables for gyro raw data
  int16_t temperature;
  int16_t x;
  int16_t y;
  int16_t loc[];

public:
  Location(int16_t x, int16_t y)
  {
    this->x = x;
    this->y = y;
    init();
  }
  void init()
  {
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
    Wire.write(0x6B);                 // PWR_MGMT_1 register
    Wire.write(0);                    // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
  }
  byte getlocation(int16_t loc[])
  {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);                        // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
    Wire.endTransmission(false);             // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
    Wire.requestFrom(MPU_ADDR, 7 * 2, true); // request a total of 7*2=14 registers

    // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
    accelerometer_x = Wire.read() << 8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
    accelerometer_y = Wire.read() << 8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
    accelerometer_z = Wire.read() << 8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
    temperature = Wire.read() << 8 | Wire.read();     // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
    gyro_x = Wire.read() << 8 | Wire.read();          // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
    gyro_y = Wire.read() << 8 | Wire.read();          // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
    gyro_z = Wire.read() << 8 | Wire.read();          // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

    loc[0] = accelerometer_x
        // ...
        return loc
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

#define leftIR A2
#define rightIR A3
  LineDetector lineDetector(leftIR, rightIR);

  byte pin = 5;
  byte initialAngle = 90;
  Gripper gripper(pin, initialAngle);

  int triggerPin = 6;
  int echoPin = 7;
  UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);

  int16_t x = 0;
  int16_t y = 0;
  Location location(x, y);

  long time;
  long timeElapsed;
  double distanceToBlock;

  // the different block-retrieval states we could be in
  enum
  {
    getMiddleBlock,
    getHomeOnLine,
    getClosestBlock,
    goToEnemyBase
  };
  unsigned char robotState = getMiddleBlock;

  ////////////////////////
  // HELPFUL FUNCTIONS //
  ///////////////////////

  void _swap(double *p, double *q)
  {
    int t;

    t = *p;
    *p = *q;
    *q = t;
  }
  LineDetector lineDetector(leftIR, rightIR);

  byte pin = 5;
  byte initialAngle = 90;
  Gripper gripper(byte pin, byte initialAngle);

  int triggerPin = 10;
  int echoPin = 11;
  UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);

  long time;
  long timeElapsed;
  double distanceToBlock;
  bool inPosition;

  // the different block-retrieval states we could be in
  enum
  {
    drivingToMiddleBlock,
    locatingBlock,
    approachingBlock,
    gettingHomeOnLine,
    getClosestBlock,
    goToEnemyBase
  };
  unsigned char robotState = drivingToMiddleBlock;

  ////////////////////////
  // HELPFUL FUNCTIONS //
  ///////////////////////

  void _swap(double *p, double *q)
  {
    int t;

    t = *p;
    *p = *q;
    *q = t;
  }

  void _sort(double a[], int n)
  {
    int i, j;

    for (i = 0; i < n - 1; i++)
    {
      for (j = 0; j < n - i - 1; j++)
      {
        if (a[j] > a[j + 1])
          _swap(&a[j], &a[j + 1]);
      }
    }
  }

  double getMedianDistance()
  {
    // denoising the data by taking median of three measurements
    int n = 3;

    double distance_1 = distanceSensor.measureDistanceCm();
    double distance_2 = distanceSensor.measureDistanceCm();
    double distance_3 = distanceSensor.measureDistanceCm();
    double distances[] = {distance_1, distance_2, distance_3};
    double getMedianDistance()
    {
      // denoising the data by taking median of three measurements
      int n = 5;

      double distance_1 = distanceSensor.measureDistanceCm();
      double distance_2 = distanceSensor.measureDistanceCm();
      double distance_3 = distanceSensor.measureDistanceCm();
      double distance_4 = distanceSensor.measureDistanceCm();
      double distance_5 = distanceSensor.measureDistanceCm();
      double distances[] = {distance_1, distance_2, distance_3, distance_4, distance_5};

      _sort(distances, n);
      n = (n + 1) / 2 - 1;

      double median_distance = distances[n];
      return median_distance;
    }

    void driveToLocation(long duration, int speed)
    {

      Serial.println("Driving to location...");
      time = millis();
      timeElapsed = 0;

      while (timeElapsed < duration)
      {
        forward(motor1, motor2, speed);
        while (timeElapsed < duration)
        {
          forward(motor1, motor2, speed);
          timeElapsed = millis() - time;
        }

        brake(motor1, motor2);
        Serial.println("Arrvived at location...");
      }
      delay(1000);
    }

    double _scanForBlockInDirection(long duration, int direction)
    {
      // direction == 1 -> left; direction == -1 -> right

      double distance;
      time = millis();
      timeElapsed = 0;

      while (timeElapsed < duration)
      {
        left(motor1, motor2, direction * 200);
        distance = getMedianDistance();

        if (distance < 15 && distance > 0)
        {
          brake(motor1, motor2);
          Serial.println("Detected object at distance:");
          Serial.println(distance);
          return distance;
        }
        while (timeElapsed < duration)
        {
          left(motor1, motor2, direction * 200);
          distance = getMedianDistance();

          if (distance < 15 && distance > 0)
          {
            brake(motor1, motor2);
            Serial.println("Detected object at distance:");
            Serial.println(distance);
            delay(1000);
            return distance;
          }
          timeElapsed = millis() - time;

          timeElapsed = millis() - time;
        }
        brake(motor1, motor2);
        return -1;
      }

      double scanForBlock(long duration)
      {

        double distance;

        Serial.println("Scanning in left direction...");
        distance = _scanForBlockInDirection(duration, 1); // scanning left

        if (!distance == -1)
        {
          if (distance > -1)
          {
            return distance; // found block on left rotation
          }

          Serial.println("No block found on left rotation. Scanning in right direction...");
          distance = _scanForBlockInDirection(duration * 2, -1);

          if (!distance == -1)
          {
            return distance; // found block on right rotation
          }

          Serial.println("No block found on right rotation...");
          return distance; // did not find block
        }
        if (distance > -1)
        {
          return distance; // found block on right rotation
        }

        Serial.println("No block found on right rotation either...");
        return distance; // did not find block
      }

      void approachBlock(double originalDistance)
      {

        int speed = 50;
        int distForGrippers = 2;
        double distance = originalDistance;

        Serial.println("Approaching block...");
        while (distance > distForGrippers)
        {
          forward(motor1, motor2, speed);

          distance = getMedianDistance();
          Serial.println("Distance to block...");
          Serial.println(distance);

          if (distance > originalDistance + 10)
          {
            Serial.println("We've lost the block!!!");
            brake(motor1, motor2);
            driveToLocation(100, -50);     // backtrack a little
            distance = scanForBlock(2000); // search for the block again
          }
        }
      }
      bool approachBlock(double originalDistance)
      {

        int speed = 50;
        int distForGrippers = 3;
        double distance = originalDistance;
        int retries = 3;

        Serial.println("Approaching block...");
        while (distance > distForGrippers)
        {

          forward(motor1, motor2, speed);
          distance = getMedianDistance();

          Serial.println("Distance to block...");
          Serial.println(distance);
          delay(1000);

          if (distance > originalDistance + 10 || distance == -1)
          {
            Serial.println("We've lost the block!!!");
            brake(motor1, motor2);
            delay(1000);
            Serial.println("Retrying three times...");
            for (int i = 0; i < retries; i++)
            {
              distance = getMedianDistance();
              if (distance > originalDistance + 10 || distance == -1)
                continue;
              else
                break;
            }
            if (distance > originalDistance + 10 || distance == -1)
            {
              Serial.println("Couldn't locate block again, beginning search...");
              driveToLocation(100, -100);    // backtrack a little
              distance = scanForBlock(2000); // search for the block again
              if (distance == -1)
                return false;
              continue;
            }
            Serial.println("Found the lost block!");
          }
        }
        brake(motor1, motor2);
        delay(1000);
        return true;
      }

      void pickUpBlock(byte desiredAngle)
      {
        byte currentAngle = gripper.getAngle(desiredAngle);
        Gripper(pin, currentAngle);
      }

      void dropBlock()
      {
        ;
      }

      void orientateOnLine()
      {
        ;
      }

      void driveAlongLine(int speed, int adjustmentSpeed)
      {

        int lineState = lineDetector.getState();

        forward(motor1, motor2, speed);

        if (lineState == 0)
          left(motor1, motor2, adjustmentSpeed);
        else if (lineState == 1)
          right(motor1, motor2, adjustmentSpeed);
      }

      void navigateAroundBlock()
      {
        ;
      }

      bool isObstacle()
      {
        double distance = getMedianDistance();
        if (distance < 15)
          return true;
        return false;
      }

      void calcLocation()
      {
        int v = 1 // e.g. 1 cm/s
            int ti = millis();
        int dur = 1000; // Duration you want to run
        double speed = Location.getlocation();

        // run motor for 'dur' amount of time
        forward(motor1, motor2, 100) while (millis() < ti + dur){
            x = x + (0.5 * speed[0] * dur * *2 + v * dur) * cos(speed[0])
            // ...
        } brake(motor1, motor2)
      }

      //////////
      // MAIN //
      //////////

      void setup()
      {
        Serial.begin(9600);
      }

      void loop()
      {

        switch (robotState)
        {

        case getMiddleBlock:
          Serial.println("Getting middle block...");

          driveToLocation(2500, 250);           // move to location near middle block
          distanceToBlock = scanForBlock(2000); // determine distance to block and align robot

          if (distanceToBlock > 0)
          {
            approachBlock(distanceToBlock); // get into position to pick up block
            pickUpBlock();                  // use grippers to pick up block
            robotState = getHomeOnLine;     // take the block home
          }
          else
          {
            ; // what to do when we don't find block after scanning left and right?
          }
          break;

        case getHomeOnLine:
          // Note: can use loop instead of sequential approach
          Serial.println("Going home along line...");

          orientateOnLine();       // align robot on the line
          driveAlongLine(200, 10); // move along line in the direction of home
          navigateAroundBlock();   // move around closest block (it is blocking path home)
          driveAlongLine(200, 10);

        default:
          break;
        }
      }
      void setup()
      {
        Serial.begin(9600);
      }

      void loop()
      {

        switch (robotState)
        {

        case drivingToMiddleBlock:
          Serial.println("Driving to middle block...");
          driveToLocation(2500, 250); // move to location near middle block
          robotState = locatingBlock;
          break;

        case locatingBlock:
          Serial.println("Locating block...");
          distanceToBlock = scanForBlock(2000); // determine distance to block and align robot
          if (distanceToBlock > 0)
          {
            robotState = approachingBlock;
            delay(1000);
          }
          else
          {
            Serial.println("No block located. Trying again.");
            delay(1000); // what to do when we don't find block after scanning left and right?
          }
          break;

        case approachingBlock:
          Serial.println("Approaching block...");
          inPosition = approachBlock(distanceToBlock); // get into position to pick up block
          if (inPosition)
            pickUpBlock();                // use grippers to pick up block
          robotState = gettingHomeOnLine; // take the block home
                                          //        what to do if we lose the block? Maybe just go home on line and go for closer block.

        case gettingHomeOnLine:
          Serial.println("Going home along line...");

          orientateOnLine();       // align robot on the line
          driveAlongLine(200, 10); // move along line in the direction of home
          navigateAroundBlock();   // move around closest block (it is blocking path home)
          driveAlongLine(200, 10);

        default:
          break;
        }
      }
