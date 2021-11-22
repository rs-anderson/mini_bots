#include <SparkFun_TB6612.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Servo.h>
#include <HCSR04.h>
// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
#define AIN1 2
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

// pings for the distance detection with ultrasound
#define triggerPin 11
#define echoPin 12

// line follower with ir detection
#define leftIR A2
#define rightIR A3

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// minimal distance for detecting blocks
const double MIN_DISTANCE_FROM_OBJECT = 5;

// stop the car if run into the other bot
const char GREEN_SIGN = 'd'; // Drive
const char RED_SIGN = 's';   // Stop
char vehicleAction;

// device tolerance
const int SENSITIVITY_OFFSET = 30;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// initializing the us
UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);

// initializing the rgb detector
Adafruit_TCS34725 tcs = Adafruit_TCS34725();

// initializing the claw
Servo servoClaw; // servo motor to open/close the claw

int leftThreshold;
int rightThreshold;
int clawAngle = 45; // servoClaw's angle initialized

void setup()
{
    Serial.begin(115200);
    pinMode(leftIR, INPUT);
    pinMode(rightIR, INPUT);
    leftThreshold = analogRead(leftIR);
    rightThreshold = analogRead(rightIR);
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
    servoClaw.attach(3);
    clawAngle = move(servoClaw, clawAngle, clawAngle);

    if (tcs.begin())
    {
        Serial.println("Found sensor");
    }
    else
    {
        Serial.println("No TCS34725 found ... check your connections");
        while (1)
            ;
    }
}

void loop()
{
    // if (Serial.available() > 0)
    // {
    //     char serial = Serial.read();

    //     if (serial == GREEN_SIGN || serial == RED_SIGN)
    //     {
    //         vehicleAction = serial;
    //     }
    // }

    // if (vehicleAction == GREEN_SIGN)
    //     drive();
    // else if (vehicleAction == RED_SIGN)
    //     brake(motor1, motor2);

    uint16_t r, g, b, c, colorTemp, lux;

    tcs.getRawData(&r, &g, &b, &c);
    //colorTemp = tcs.calculateColorTemperature(r, g, b);
    //lux = tcs.calculateLux(r, g, b);

    //Serial.print("Color Temp: ");
    // Serial.print(colorTemp, DEC);
    // Serial.print(" K - ");
    // Serial.print("Lux: ");
    // Serial.print(lux, DEC);
    // Serial.print(" - ");
    Serial.print("R: ");
    Serial.print(r, DEC);
    // Serial.print(" ");
    Serial.print("G: ");
    Serial.print(g, DEC);
    // Serial.print(" ");
    Serial.print("B: ");
    Serial.print(b, DEC);
    // Serial.print(" ");
    // Serial.print("C: ");
    // Serial.print(c, DEC);
    // Serial.print(" ");
    // Serial.println(" ");
    if (r == 0 & g == 0 & b == 0)
    {
        clawAngle = move(servoClaw, clawAngle, 90);
        delay(1000);
    }

    // detect the homebase and drop it
    // clawAngle = move(servoClaw, clawAngle, 45);
    // delay(1000);
}

void drive()
{
    if (hasObstacle())
        brake(motor1, motor2);
    while (hasObstacle())
        delay(100);

    // If a new vehicle action was received, must process it first
    if (Serial.available() > 0)
        return;

    int leftIrValue = analogRead(leftIR) - SENSITIVITY_OFFSET;
    int rightIrValue = analogRead(rightIR) - SENSITIVITY_OFFSET;

    if (leftIrValue <= leftThreshold && rightIrValue <= rightThreshold)
        forward(motor1, motor2, 150);
    else if (leftIrValue > leftThreshold && rightIrValue <= rightThreshold)
        left(motor1, motor2, 100);
    else if (leftIrValue <= leftThreshold && rightIrValue > rightThreshold)
        right(motor1, motor2, 100);
    else if (leftIrValue > leftThreshold && rightIrValue > rightThreshold)
        brake(motor1, motor2);
}

boolean hasObstacle()
{
    double distance = distanceSensor.measureDistanceCm();

    // Check if distance is close enough for block detection as well as the other bot
    return distance > 0 && distance <= MIN_DISTANCE_FROM_OBJECT;
}

int move(Servo servoTemp, int currentAngle, int desiredAngle)
{
    if (currentAngle < desiredAngle)
    {
        while (currentAngle <= desiredAngle)
        {
            servoTemp.write(currentAngle);
            currentAngle++;
            delay(10);
        }
    }
    else
    {
        while (currentAngle >= desiredAngle)
        {
            servoTemp.write(currentAngle);
            currentAngle--;
            delay(10);
        }
    }

    return currentAngle;
}