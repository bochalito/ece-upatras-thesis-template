#include <Servo.h>

Servo Rudder;

// This is the final output
// written to the motor.
String incomingString;

int pre_val = 1500;

void setup() {

  //pinMode(1,OUTPUT);
  Rudder.attach(10); //analog pin 1
  Serial.begin(19200);
  Serial.println("Ready");
        
          Rudder.writeMicroseconds(1500); 
          delay(2000);
          Rudder.writeMicroseconds(1550); 
          delay(2000);
           Rudder.writeMicroseconds(1600); 
          delay(2000);               
}

void loop() {

  if ( Serial.available()) {
    char ch = Serial.read();
    
    if (ch != 10){
      incomingString += ch;
    }   
    else {
      int val = incomingString.toInt();
     
        // print the integer
        //Serial.print("Printing the value: ");

        if ( (val <2000) && (val>1000) ) {
          Serial.println(val);
          Rudder.writeMicroseconds(val); 
          delay(1000);
          pre_val=val;
        }
        incomingString = "";
    }   
  }
} 
