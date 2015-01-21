#define pinUD 15    // A0 is pin 15
#define pinLR 14    // A1 is pin 14

#include <Wire.h>
#include <Servo.h>

Servo servoUD;

int offsetUD, offsetLR;
int valUD, valLR;

void setup() {
  Serial.begin(9600);
  offsetUD = analogRead(pinUD);
  offsetLR = analogRead(pinLR);
  Serial.println("Joystick ready.");
  
  servoUD.attach(9);
  servoUD.writeMicroseconds(1000);
  delay(2000);
  Serial.println("Up/Down Servo ready.");
}

void loop() {
  valUD = analogRead(pinUD)-offsetUD;
  valLR = analogRead(pinLR)-offsetLR;
  
  if(abs(valUD) > 5 || abs(valLR) > 5){
    Serial.print( valUD );
    Serial.print(" ");
    Serial.println( valLR );
    servoUD.writeMicroseconds(1000+valUD);
  } else {
    // Stop the motor.
    servoUD.writeMicroseconds(1000);
  }
}
