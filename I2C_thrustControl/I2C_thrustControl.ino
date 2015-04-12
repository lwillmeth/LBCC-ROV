#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
//#include <Servo.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define DEBUGGING true    // Enables debugging output
#define ECHO_ON true      // Enables echoing commands back to controller
#define STOP 500      // Sets the stop position for the ESC
#define TIMEOUT 3000   // Sets timeout in case communication to surface is lost

#define PROP_MAX 500    // Largest acceptable control value
#define PROP_MIN -500

#define CTRL_SIZE 12   // Number of devices we're controlling
#define STOP 0      // The prop ESC's are set to stop at 1000 us

#define UDP_BUFFER_SIZE 36 // 3 bytes * 12 servos = 36 byte buffer

/* ============================================================================
  Settings
============================================================================ */

// Ethernet variables
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip		(10,1,1,1);
IPAddress gateway	(10,0,0,1);
IPAddress subnet	(255,0,0,0);
EthernetUDP Udp;
uint16_t localPort = 21050;
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

/* ============================================================================
  Main
============================================================================ */

void setup(){
  Serial.begin(9600);
  Serial.println(F("Waiting for ethernet..."));
  // Begin Ethernet
  if (Ethernet.begin(mac) == 0) {
    // DHCP will timeout after 1 minute.
    Ethernet.begin(mac, ip, gateway, subnet);
  }
  Serial.println(Ethernet.localIP() );
  Udp.begin(localPort);
  if(DEBUGGING){
    Serial.print(F("IP: "));
    Serial.print(Ethernet.localIP());
    Serial.print(F(":"));
    Serial.println(localPort);
  }
  
  // Begin I2C
  pwm.begin();
  pwm.setPWMFreq(1540);  // Min PWM freq is 40, and this is the max.

  // Arm all motors by sending each of them a STOP position.
  stopMotors();
}

void loop (){
  // Listen for, and process incoming commands.
  handleIncomingData();
  // Full stop if we lose connection to command station.
  if( millis() - lastCom > TIMEOUT){
    // Send an error message to anyone listening.
    broadcast(TIMEOUT);
    // Stop all motors.
    stopMotors();
    delay(500);
  }
}

/* ============================================================================
  Motor Control
============================================================================ */
void stopMotors (){
  // Sends a STOP position to each of the prop motors
  for(int8_t i=0; i < 8; i++){
    setValue(i, STOP);
  }
  // Send error code in case anyone's listening.
  sendUDP(30, 99);
}

/* ============================================================================
  Networking
============================================================================ */

void handleIncomingData(){
  // Read incoming ethernet data and interpret commands.
  // Find size of incoming datagram packet.
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
      Serial.print(" ");
      Serial.println((String)udp_buffer);
    }
    if(ECHO_ON){
      // Send the buffer back for debugging
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(udp_buffer);
      Udp.endPacket();
    }

    // Read first byte in the buffer, interpret as command code:
    switch(udp_buffer[0]){
      case 1:
        Serial.println("Set cmd received.");
        // SET [num_changes] [ind_1] [val_1] [ind_2] [val_2] etc
        // Read next 3 bytes of buffer, interpret as SET [index] [value]
        for(int i=1; i+2 < packetSize; i+=2){
          // Read next 3 bytes of buffer, interpret as SET [index] [value]
          // Bit shift and logical OR recombine bytes 2&3 into an int
          Serial.println( (byte)udp_buffer[i]);
          Serial.println( (byte)udp_buffer[i+1]);
          Serial.println( (byte)udp_buffer[i+2]);
          Serial.println( (byte)udp_buffer[i+3]);
          Serial.println( udp_buffer[i+1]*256 + udp_buffer[i+2] );
          Serial.println( (int)( (udp_buffer[i+1]<< 8) + udp_buffer[i+2]) );
          setValue( udp_buffer[i], int(udp_buffer[i+1] + udp_buffer[i+2]) );
          //Udp.write(index & value);  // Is a reply even necessary?
        }
        break;
      case 2:
        // GET command returns a motor(s) current speed setting.
        // GET [index1] [index2] [index3] etc.
        for(int i=0; i<(packetSize-8); i++){
          getValue( udp_buffer[i]);
        }
        break;
      case 3:
        // RANGE command, returns min and max motor values
        // RANGE [index1] [index2] [index3] etc.
        for(int i=0; i<(packetSize-8); i++){
          getRange( udp_buffer[i]);
        }
        break;
      default:
        // Signals an 'invalid input' error.
        if(DEBUGGING){
          Serial.print("Error: Invalid CMD: '");
          Serial.print(udp_buffer[0]);
          Serial.println("'");
        }
        if(ECHO_ON) sendUDP(0, 999);
    }
    // Update timer each time we receive control data
    lastCom = millis();
  }
}

void broadcast (int gap){
  static const IPAddress BROADCAST_IP (255,255,255,255);

  if(DEBUGGING){
    Serial.println(F("Broadcast!"));
    Serial.print(F("IP: "));
    Serial.println(ip_to_str(BROADCAST_IP));
    Serial.print(F("PORT: "));
    Serial.println(localPort, DEC);
  }
  Udp.beginPacket(BROADCAST_IP, localPort);
  Udp.write(byte(1));
  Udp.write(byte(2));
  Udp.write(int(100));
  Udp.endPacket();
  delay(500);
}

void setValue (int8_t index, int16_t value){
  Serial.println("Processing SET.");
  // Takes a (byte)index and (int)value, stores them in control array.
  // Check if value is within acceptable range, otherwise return an error.
  if(index >= 0 && index <= CTRL_SIZE && value <= PROP_MAX && value >= PROP_MIN){
    if(DEBUGGING){
      Serial.print("Setting index: ");
      Serial.print(index);
      Serial.print(" to value: ");
      Serial.println(value);
    }
    // -520 to 520, motors ignore small amounts over abs(500)
    control[index] = value;
    // Should be able to write value once and let I2C store it.
    pwm.setPWM(index, 0, control[value]);
    
    // Send back confirmation of new value.. may disable later.
    getValue(index);
  } else {
    // Invalid SET cmd, send back an error.
    if(DEBUGGING){
      Serial.print("Invalid SET index: ");
      Serial.print(index);
      Serial.print(" or value: ");
      Serial.println(value);
    }
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
