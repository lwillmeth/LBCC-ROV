#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Servo.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define DEBUGGING true    // Enables debugging output
#define ECHO_ON true      // Enables echoing commands back to controller
#define STOP 1000      // Sets the stop position for the ESC
#define TIMEOUT 3000   // Sets timeout in case communication to surface is lost

#define PROP_MAX 500    // Largest acceptable control value
#define PROP_MIN -500

#define CTRL_SIZE 12   // Number of devices we're controlling
#define STOP 1000      // The prop ESC's are set to stop at 1000 us

#define UDP_BUFFER_SIZE 36 // 3 bytes * 12 servos = 36 byte buffer
// slipperyrobot
/* ============================================================================
  Settings
============================================================================ */
// Ethernet variables
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip		(10,1,1,1);
IPAddress gateway	(10,0,0,1);
IPAddress subnet	(255,0,0,0);
EthernetUDP Udp;
uint16_t localPort = 21015;
uint32_t lastCom;
char udp_buffer[UDP_BUFFER_SIZE];

// Prepare I2C Shield
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// An array to hold our prop speeds and servo positions
int16_t control[CTRL_SIZE];

int16_t servoRange[][3] = {
  // Holds min, max, default servo positions.  Expand this later.
  {0, 180, 90},
  {0, 180, 90},
  {0, 180, 90},
  {0, 180, 90}
};
int motor_rpm = 0;

// An array of servo objects, one for each prop and servo.
Servo motors[CTRL_SIZE];

/* ============================================================================
  Main
============================================================================ */

void setup (){
  Serial.begin(9600);

  // Begin Ethernet
  if (Ethernet.begin(mac) == 0) {
    // DHCP will timeout after 1 minute.
    Ethernet.begin(mac, ip, gateway, subnet);
  }
  Udp.begin(localPort);
  Serial.print(F("IP: "));
  Serial.print(Ethernet.localIP());
  Serial.print(F(":"));
  Serial.println(localPort);

  // Begin I2C
  pwm.begin();
  pwm.setPWMFreq(1500);  // This is the maximum PWM frequency
  uint8_t twbrbackup = TWBR;
  TWBR = 12;
  
  // Designate a pin for each prop, allowing us to control them
  motors[0].attach(2);
  motors[1].attach(3);
  motors[2].attach(4);
  motors[3].attach(5);
  // Add rest of motors as we assign pins.

  // Prop motors must be 'armed' by sending a "stop" position
  motors[0].writeMicroseconds(STOP);
  motors[1].writeMicroseconds(STOP);
  motors[2].writeMicroseconds(STOP);
  motors[3].writeMicroseconds(STOP);
}

void loop (){
  handleIncomingData();
  if(DEBUGGING){
    broadcast(1000);
  }
  // Full stop if we lose connection to command station.
  if( micros() - lastCom < TIMEOUT){
    motor_rpm++;
    runMotors();
  } else {
    stopMotors();
  }
}

/* ============================================================================
  Motor Control
============================================================================ */

void runMotors (){
  // For now, only testing first 4 props.  Later, we can use i<CTRL_SIZE to run them all.
  for(int8_t i=0; i<5; i++){
    /*
    More thought should go into this.
    We have an array of motors, most of which won't be moving.
    It would be wise to skip non-moving motors, without requiring they
    have a control value of 0.  I.e. a servo which may or may not rest
    at a 0 position.
    One solution may be to use an array of booleans,
    or even a bit from a double, to signal active motors?
    */
    if(control[i] != 0){
//      motors[i].writeMicroseconds(control[i]);
    }
  }
}

void stopMotors (){
  // Failsafe if communication with surface is lost.
  // Set all motors and servos to default STOP positions.
  for(int8_t i=0; i<CTRL_SIZE; i++){
    control[i] = STOP;
  }
  // Send error code in case anyone's listening.
//  sendUDP(30, 99);
}

/* ============================================================================
  Networking
============================================================================ */

void handleIncomingData (){
  // Read incoming ethernet data and interpret commands.
  // static char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
  // static char replyBuffer[] = "acknowledged";
  int packetSize = Udp.parsePacket();
  if (packetSize){
    // Read incoming packet into buffer.
    Udp.read(udp_buffer, UDP_BUFFER_SIZE);
    if(DEBUGGING){
      Serial.print(F("Received packet of size "));
      Serial.print(packetSize);
      Serial.print(F(" from "));
      Serial.print(ip_to_str(Udp.remoteIP()));
      Serial.print(F(", port "));
      Serial.println(Udp.remotePort());
      Serial.print("Motor rpm: ");
      Serial.print(motor_rpm);
      Serial.print(" ");
      Serial.println((String)udp_buffer);
    }
    sendUDP(1, 2, motor_rpm);
    motor_rpm = 0;
//    if(ECHO_ON){
//      // Echo the buffer back
//      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
//      Udp.write(udp_buffer);
//      Udp.endPacket();
//    }

    // Read one byte from the buffer, interpret as command code:
    switch( udp_buffer[0] ){
      case 1:
        // SET command, update control value based on index
        /* This process could be improved!
        Read in a byte for the index, then smash two bytes together for value.
        The problem is Udp.read() can only read one byte without using a buffer.
        Either we read all incoming data on-the-fly, or store them all in advance.
        I'd rather avoid the overhead of reading them in advance, but this seems messy.
        */
        //setValue( Udp.read(), (int)(Udp.read()<<Udp.read()) );
        
        // NEW METHOD (needs testing)
        // Number of changes determined by size of packet, adjusted for overhead
        // Read next byte of buffer, interpret as SET [index]
        for(int i=0; i<(packetSize-8); i+=3){
          // Read next 3 bytes of buffer, interpret as SET [index] [value]
          // Bit shift and logical OR recombine bytes 2&3 into an int
          setValue( udp_buffer[i], (udp_buffer[i+1]<<8 | udp_buffer[i+2]) );
          //Udp.write(index & value);  // Is an echo even necessary?
        }
        break;
      case 2:
        // GET command, return current value based on index
        //getValue( Udp.read() );
        
        // NEW METHOD using SET [index1] [index2] [index3] etc.
        for(int i=0; i<(packetSize-8); i++){
          // Read next byte of buffer, interpret as GET [index1] [index2]
          getValue( udp_buffer[i]);
        }
        break;
      case 3:
        // RANGE command, returns min and max motor values
        //getRange( Udp.read() );
                
        // NEW METHOD using RANGE [index1] [index2] [index3] etc.
        for(int i=0; i<(packetSize-8); i++){
          // Read next byte of buffer, interpret as RANGE [index1] [index2]
          getRange( udp_buffer[i]);
        }
        break;
      default:
        // Signals an 'invalid input' error.
        sendUDP(0, 0);
    }
    // Update timer each time we receive control data
    lastCom = micros();
  }
}

void broadcast (int gap){
  static unsigned long last = -gap;
  static const IPAddress BROADCAST_IP (255,255,255,255);
  static const int BROADCAST_PORT (21025);

  if (millis() - last > gap) {
    last = millis();
//    Serial.println(F("Broadcast!"));
//    Serial.print(F("IP: "));
//    Serial.println(ip_to_str(BROADCAST_IP));
//    Serial.print(F("PORT: "));
//    Serial.println(BROADCAST_PORT, DEC);

    Udp.beginPacket(BROADCAST_IP, BROADCAST_PORT);
    Udp.write(byte(1));
    Udp.write(byte(2));
    Udp.write(int(100));
    Udp.endPacket();
  }
}

void setValue (int8_t index, int16_t value){
  // Takes a (byte)index and (int)value, stores them in control array.
  // Check if value is within acceptable range, otherwise return an error.
  if(index >= 0 && index <= CTRL_SIZE && value <= PROP_MAX && value >= PROP_MIN){
    // -520 to 520, motors ignore small amounts over abs(500)
    control[index] = value*4;
    // Send back confirmation of new value.. may disable later.
    //getValue(index);
  } else {
    // Invalid SET cmd, send back an error.
    sendUDP(1, index);
  }
}

void getValue (int8_t index){
  // Sends the current value of a control index along the UDP connection.
  if(index <= CTRL_SIZE && index >= 0){
    // Send back current value of a prop/servo.
    sendUDP(2, index, control[index]);
  } else {
    // Invalid GET cmd, send back an error.
    sendUDP(2, index);
  }
}

void getRange (int8_t index){
  // Returns [cmd] [index] [min] [max] [default] motor/servo values.
  if(index >= 0 && index < 8){
    // All props share the same min and max values.
    sendUDP(3, index, PROP_MIN, PROP_MAX, STOP);
  } else if(index <= CTRL_SIZE){
    // Each servo may have a unique start and stop position.
    sendUDP(3, index, servoRange[index][0],
                      servoRange[index][1],
                      servoRange[index][2]);
  } else {
    // Index out of bounds, send back an error.
    sendUDP( 0, 3);
  }
}
  
void sendUDP (int8_t code, int8_t index, int8_t value){
  // Sends back data in a format consistant with standards.md protocal.
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(code);
  Udp.write(index);
  Udp.write(value);
  Udp.endPacket();
}

void sendUDP (int8_t code, int8_t index, int16_t valueA, int16_t valueB, int16_t valueC){
  // Overloaded method for a code and two integers.
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(code);
  Udp.write(index);
  Udp.write(valueA);
  Udp.write(valueB);
  Udp.write(valueC);
  Udp.endPacket();
}

void sendUDP (int8_t code, int8_t index){
  // Overloaded method for a code and index pair with no value.
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(code);
  Udp.write(index);
  Udp.endPacket();
}

/* ============================================================================
  Debugging
============================================================================ */

const char* ip_to_str (const IPAddress ipAddr){
  static char buf[16];
  sprintf(buf, "%d.%d.%d.%d\0", ipAddr[0], ipAddr[1], ipAddr[2], ipAddr[3]);
  return buf;
}
