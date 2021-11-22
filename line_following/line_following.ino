/* 
 Demo code for using IR_sensors.
 Datasheet for TCRT5000
 *  
 *  
 *   
 */
#include <SparkFun_TB6612.h>
// white = 0
// black = 1
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

#define leftIR A0
#define rightIR A1
//uses A2, A3 as in

void setup() {
  // put your setup code here, to run once:
  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);
  Serial.begin(9600);
}

void loop() {
  forward(motor1, motor2, 150);
  
  // put your main code here, to run repeatedly:
  if( leftIR == 1 && rightIR == 0) 
  left(motor1, motor2, 100);
  
  else if( leftIR == 0 && rightIR == 1)
  right(motor1, motor2, 100);
  delay(1000);

}
