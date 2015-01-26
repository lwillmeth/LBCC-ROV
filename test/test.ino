/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  PWM test - this will drive 16 PWMs in a 'wave'

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN 150
#define SERVOMAX 600

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setPWMFreq(60);  // This is the maximum PWM frequency
//    
//  // save I2C bitrate
//  uint8_t twbrbackup = TWBR;
//  // must be changed after calling Wire.begin() (inside pwm.begin())
//  TWBR = 12; // upgrade to 400KHz!
}

void loop() {
  // Drive each PWM in a 'wave'
  for (uint8_t motor=0; motor<8; motor++) {
    Serial.print("Motor #");
    Serial.println(motor);
    
    Serial.println("Forward..");
    for(uint16_t pulselen=SERVOMIN; pulselen<SERVOMAX; pulselen++){
      pwm.setPWM(motor, 0, pulselen);
    }
    delay(500);

    Serial.println("Reverse..");    
    for(uint16_t pulselen=SERVOMAX; pulselen>SERVOMIN; pulselen--){
      pwm.setPWM(motor, 0, pulselen);
    }
  }
}
