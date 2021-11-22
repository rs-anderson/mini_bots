// code by Aryaman Kumar

#include <Servo.h>

Servo servoClaw;    // servo motor to open/close the claw 
Servo servoHeight;  // servo motor to raise/lower the claw 
Servo servoTurn;    // servo motor to turn the claw 


int clawAngle = 70;   // servoClaw's angle initialized 
//int heightAngle = 160; // servoHeight's angle initialized 
//int turnAngle = 0;     // servoTurn's angle initialized 

// refer the definition of function "move()" below to understand the code in "setup()"

void setup() {
  
   Serial.begin (9600);
   // pin mapping
   servoClaw.attach(12);  
   //servoHeight.attach(4);
   //servoTurn.attach(5);

   // Setting all servo motors to initial angles. Claw is now right above cup.
   
   //heightAngle = move(servoHeight,heightAngle,heightAngle);
   //turnAngle = move(servoTurn,turnAngle,turnAngle);
   clawAngle = move(servoClaw,clawAngle,clawAngle);
 
   // moving towards first object and picking it up

   //turnAngle = move(servoTurn,turnAngle,90);
   //delay(1000);
   //heightAngle = move(servoHeight,heightAngle,90);
   //delay(1000);
   clawAngle = move(servoClaw,clawAngle,10);
   delay(1000);
   //heightAngle = move(servoHeight,heightAngle,160);
   //delay(1000);
    
   // moving back to cup and dropping the object
     
//   turnAngle = move(servoTurn,turnAngle,0);
//   delay(1000);
//   heightAngle = move(servoHeight,heightAngle,140);
//   delay(1000);
//   clawAngle = move(servoClaw,clawAngle,150);
//   delay(1000);
//   heightAngle = move(servoHeight,heightAngle,160);
//   delay(1000);
//
//   // moving towards second object and picking it up
//    
//   turnAngle = move(servoTurn,turnAngle,160);
//   delay(1000);
//   heightAngle = move(servoHeight,heightAngle,90);
//   delay(1000);
//   clawAngle = move(servoClaw,clawAngle,10);
//   delay(1000);
//   heightAngle = move(servoHeight,heightAngle,160);
//   delay(1000);
//
//   // moving back to cup and dropping the object
//       
//   turnAngle = move(servoTurn,turnAngle,0);
//   delay(1000);
//   heightAngle = move(servoHeight,heightAngle,140);
//   delay(1000);
//   clawAngle = move(servoClaw,clawAngle,150);
//   delay(1000);
//   heightAngle = move(servoHeight,heightAngle,160);
//   delay(1000);
     
}

// No code in loop because it is required to operate only once. 
void loop() {
   
}

// servoTemp is the servo that is to be moved. 
// currentAngle is the servo motor's current angle.
// desiredAngle is the angle it is required to move to.

// this function moves the servo from its current angle to the desired angle. Returns new angle. 


int move(Servo servoTemp,int currentAngle, int desiredAngle)
{
  if(currentAngle<desiredAngle)
  {
     while(currentAngle<=desiredAngle)
    {
      servoTemp.write(currentAngle);
      currentAngle++;
      delay(10);
    }  
  }
  else
  {
     while(currentAngle>=desiredAngle)
    {
      servoTemp.write(currentAngle);
      currentAngle--;
      delay(10);
    }  
  }

   return currentAngle; 

}
