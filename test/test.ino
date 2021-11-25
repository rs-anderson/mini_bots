#include <Servo.h>
#include <SparkFun_TB6612.h>
#include <TimeLib.h>
#include <HCSR04.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>

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

        loc[0] = accelerometer_x;
            // ...
            return loc;
    }
};

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


    byte pin = 5;
    byte initialAngle = 90;
    Gripper gripper(byte pin, byte initialAngle);


    //int16_t x = 0;
    //int16_t y = 0;
    Location location(int16_t x, int16_t y);

    void pickUpBlock(byte desiredAngle)
    {
        byte currentAngle = Gripper.getAngle(desiredAngle);
        Gripper(pin, currentAngle);
    }

    void calcLocation()
    {
        int v = 1; // e.g. 1 cm/s
            int ti = millis();
        int dur = 1000; // Duration you want to run
        double speed = location.getlocation();

        // run motor for 'dur' amount of time
        forward(motor1, motor2, 100) while (millis() < ti + dur){
            x = x + (0.5 * speed[0] * dur * *2 + v * dur) * cos(speed[0])
            // ...
        }; brake(motor1, motor2);
    }
    void setup()
    {
        Serial.begin(9600);
    }
    void loop()
    {
        forward(motor1, motor2, 100);
        pickUpBlock(90);
    }
