#include <Servo.h>

#define pi 3.14159265358979323846  // close enough
#define DEBUGGING 1
#define STOP 1000

// Control inputs from the joystick
#define pin_MV_UD 15    // A0 is pin 15
#define pin_MV_LR 14    // A1 is pin 14

// Assign props to specific pins
//#define pin_prop_0 0        // Prop 0 is on pin 0
//#define pin_prop_1 1        // Prop 1 to pin 1, etc.
//#define pin_prop_2 2
//#define pin_prop_3 3

// Create servo objects for each prop
Servo prop_0;
Servo prop_1;
Servo prop_2;
Servo prop_3;

// Offsets allow us to adjust for inaccuracies in the joystick position
int offset_MV_UD, offset_MV_LR;

// Variables to store our intended direction
int MV_UD, MV_LR;
double MV_resultant, MV_theta;
int MV_A, MV_B;

void setup() {
  Serial.begin(9600);
  Serial.print("Preparing joystick...  ");
  // Offsets read in once at startup, be certain joystick is neutral.
  offset_MV_UD = analogRead(pin_MV_UD);
  offset_MV_LR = analogRead(pin_MV_LR);
  Serial.println("Joystick ready.");

  Serial.print("Preparing props...  ");
  // Attach servo objects to each prop's pin, allowing us to control the prop.
  prop_0.attach(2);
  prop_1.attach(3);
  prop_2.attach(4);
  prop_3.attach(5);

  // Arm all props by sending a "stop" position
  // "Stop" is halfway between full reverse (500), and full forward (1500).
  prop_0.writeMicroseconds(STOP);
  prop_1.writeMicroseconds(STOP);
  prop_2.writeMicroseconds(STOP);
  prop_3.writeMicroseconds(STOP);
  Serial.println("Props armed and ready.");
  
  //  servoUD.attach(9);
  //  servoUD.writeMicroseconds(1000);
  delay(2000);
}

void loop() {
  // Read in current position of joystick as x, y coordinates, then adjust for offset.
  MV_UD = analogRead(pin_MV_UD) - offset_MV_UD;
  MV_LR = analogRead(pin_MV_LR) - offset_MV_LR;

  // Ignore tiny joystick positions to prevent twitching/wasted movement
  if (abs(MV_UD) > 5 || abs(MV_LR) > 5) {
    // find A,B components by rotating axis CW pi/4 rad
    MV_resultant = sqrt( pow(MV_UD,2)+pow(MV_LR,2) );
//    Serial.println( sqrt( pow(MV_UD,2)+pow(MV_LR,2) ));
    MV_theta = atan2(MV_UD, MV_LR)-pi/4;
    
    // Find values for each set of motors, to reach target vector
    // B = motors(0,2), A = motors(1,3)
    MV_A = MV_resultant*sin(MV_theta);
    MV_B = MV_resultant*cos(MV_theta);
    
    if(DEBUGGING){
      Serial.print("Joystick=(");
      Serial.print( MV_LR );
      Serial.print(",");
      Serial.print( MV_UD );
      Serial.print(")   \tMV_resultant=");
      Serial.print( MV_resultant );
      Serial.print("\tMV_theta=");
      Serial.print( MV_theta );
      Serial.print("\tMV_A=");
      Serial.print( MV_A );
      Serial.print(" \tMV_B=");
      Serial.println( MV_B );
    }
    // Move A motors (0 & 2)
    prop_0.writeMicroseconds(STOP + MV_B);
    prop_2.writeMicroseconds(STOP + MV_B);
    // Move B motors (1 & 3)
    prop_1.writeMicroseconds(STOP + MV_A);
    prop_3.writeMicroseconds(STOP + MV_A);
  } else {
    // Move A motors (0 & 2)
    prop_0.writeMicroseconds(STOP);
    prop_2.writeMicroseconds(STOP);
    // Move B motors (1 & 3)
    prop_1.writeMicroseconds(STOP);
    prop_3.writeMicroseconds(STOP);
  }
}
