#include <SparkFun_TB6612.h>

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

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

void setup()
{
    //Nothing here
}

void loop()
{
    forward(motor1, motor2, 150);
    delay(1000);

    brake(motor1, motor2);
    delay(2000);

    left(motor1, motor2, 100);
    delay(1000);

    forward(motor1, motor2, 150);
    delay(1000);
}