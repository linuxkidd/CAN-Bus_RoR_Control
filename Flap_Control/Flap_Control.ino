/*
   Astro-Shed Flap Control
     by: Michael J. Kidd <linuxkidd@gmail.com>
    Rev: 2.0
   Date: 2020-10-11
*/

#include <SPI.h>

#include "mcp_can.h"

boolean debug                    = false;  // Enable debug output

//const int CAN_SPEED            = CAN_250KBPS;    // For Uno  CAN-Bus Shield    (16MHz)
const int CAN_SPEED              = CAN_8M_250KBPS; // For nano CAN SPI interface  (8MHz)
const int SPI_CS_PIN             = 10;           // Which pin to use for SPI CS with CAN bus interface

const int FLAP_DIRECTION_PIN     = 9;
const int FLAP_ENABLE_PIN        = 8;

const int FLAP_MOVE_DETECT_PIN   = 7;


/*
 * Variables below this point are used internally and should not be changed.
 */
INT32U status_every               =  1000;  // Send status every 1 second ( 1000 ms )
INT32U controller_status_timeout  = 15000;  // If no controller heartbeat for 15 seconds, close the roof
INT32U roof_status_timeout        = 15000;  // If no flap status for 15 seconds, close the roof

unsigned char len = 0;
unsigned char buf[8];

unsigned int rxId       = 0x000;
unsigned int roofId     = 0x200;
unsigned int flapId     = 0x300;
unsigned int roofTrigId =  0x64;
unsigned int flapTrigId =  0x65;

bool lastMovePinState=LOW;
bool MovePinState=LOW;
bool autoPulseCalib = false;

void printPacket(unsigned int myrxId, unsigned int mylen, unsigned char mybuf[8]);
void parseInstruction();
void sendStartup();
void sendState();
void readPins();
void stopMotion();
void openFlap();
void closeFlap();

MCP_CAN CAN(SPI_CS_PIN);         // Set CS pin

typedef struct {
  INT32U transmitted = 0;
  INT32U controller_received = 0;
  INT32U roof_received = 0;
} statustiming;

statustiming mystat;

typedef struct {
  boolean isOpen = false;
  boolean isClosed = false;
  boolean isOpening = false;
  boolean isClosing = false;
  unsigned int desiredState = 0; // 0 == closed, 1 == open, 2 == stopped
  INT32U lastPulse = 0;
  float extended=12;
  float OpenPulseLen=0.007046/2;  // Half to count only rising or falling, but not both
  float ClosePulseLen=0.007046/2; // Half to count only rising or falling, but not both
} flapstruct;

flapstruct flap;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Initialization");
  pinMode(FLAP_DIRECTION_PIN, OUTPUT);
  digitalWrite(FLAP_DIRECTION_PIN,1);
  pinMode(FLAP_ENABLE_PIN, OUTPUT);
  digitalWrite(FLAP_ENABLE_PIN,0);
  pinMode(FLAP_MOVE_DETECT_PIN, INPUT_PULLUP);
  Serial.println("PIN Config complete");

START_INIT:
  Serial.println("Start CAN config");
  if (CAN_OK == CAN.begin(CAN_SPEED))  {
    CAN.init_Mask(0, 1, 0);
    CAN.init_Mask(1, 1, 0);
    for (int i = 0; i < 6; i++) {
      CAN.init_Filt(i, 1, 0);
    }
    Serial.println("CAN Config Complete");
    if ( debug ) {
      Serial.println("Debugging Enabled...");
      Serial.println("Time\t\tPGN\t\t\tByte0\tByte1\tByte2\tByte3\tByte4\tByte5\tByte6\tByte7");
    }
    sendStartup();

  }
  else
  {
    delay(100);
    goto START_INIT;
  }
}

void loop() {

  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&len, buf);
    rxId = CAN.getCanId();

    if (rxId == flapTrigId)
      parseInstruction();

    if (rxId == roofId)
      mystat.roof_received=millis();

    if(debug) {
      Serial.print("Received: ");
      printPacket(rxId,len,buf);
    }
  }

  readPins();
  if( mystat.controller_received + controller_status_timeout < millis() && !flap.isClosed && !flap.isClosing ) {
    CAN.sendMsgBuf(roofTrigId,1,1,0); // Roof Close Command
    closeFlap();
  }

  if( mystat.roof_received + roof_status_timeout < millis() && !flap.isClosed && !flap.isClosing ) {
    CAN.sendMsgBuf(roofTrigId,1,1,0); // Roof Close Command, just in case it's still listening
    closeFlap();
  }

  if( mystat.transmitted + status_every<millis() )
    sendState();
}

void printPacket(unsigned int myrxId, unsigned int mylen, unsigned char mybuf[8]) {
  Serial.print(millis());
  Serial.print("\t\t");
  Serial.print("0x");
  Serial.print(myrxId, HEX);
  Serial.print("\t\t");

  for(int i = 0; i<mylen; i++) {   // print the data
    if(mybuf[i] > 15){
      Serial.print("0x");
      Serial.print(mybuf[i], HEX);    
    } else{
      Serial.print("0x0");
      Serial.print(mybuf[i], HEX);
    }      
    Serial.print("\t");            
  }
  Serial.println();
}

void sendStartup() {
  unsigned char stmp[1] = {0xff};
  if(debug) {
    Serial.write("Sending State: ");
    printPacket(flapId+1,1,stmp);
  }
  CAN.sendMsgBuf(flapId+1,1,1,stmp);
}

void sendState() {
  int mypos=flap.extended*21.25;
  unsigned char stmp[6] = {0x00, 0x00, 0x00, 0x00, 0x00, (unsigned char)mypos};
  if(flap.isOpen)
    stmp[0]=1;
  if(flap.isClosed)
    stmp[1]=1;
  if(flap.isOpening)
    stmp[2]=1;
  if(flap.isClosing)
    stmp[3]=1;
  if(flap.desiredState)
    stmp[4]=flap.desiredState;
  if(debug) {
    Serial.write("Sending State: ");
    printPacket(flapId,6,stmp);
  }
  CAN.sendMsgBuf(flapId,1,6,stmp);
  mystat.transmitted=millis();
}

void readPins() {
  MovePinState=digitalRead(FLAP_MOVE_DETECT_PIN);
  if(MovePinState != lastMovePinState) {
    lastMovePinState=MovePinState;
    flap.lastPulse=millis();
    if(flap.isOpening==true)
      flap.extended-=flap.OpenPulseLen;
    else
      flap.extended+=flap.ClosePulseLen;
    if(flap.extended>12)
      flap.extended=12;
    if(flap.extended<0)
      flap.extended=0;
    if(debug) {
      Serial.print("Pulse: ");
      Serial.print(flap.lastPulse);
      Serial.print(", ");
      
      Serial.print(flap.extended);
      Serial.print(", isOpening: ");  Serial.print(flap.isOpening);
      Serial.print(", isClosing: ");  Serial.print(flap.isClosing);
      Serial.println();
    }
  } else {
    if(flap.lastPulse+500<millis()) {
      if(debug && (flap.isOpening || flap.isClosing))
        Serial.println("Was Opening/Closing, but no pulses in 500ms.");
      if(flap.isOpening) {
        stopMotion();
        if(flap.extended!=0 && autoPulseCalib) {
          float newPulseLen=12/((12-flap.extended)/flap.OpenPulseLen);
          flap.OpenPulseLen=newPulseLen;
          if(debug) {
            Serial.print("New Open Pulse Length Calibration: ");
            Serial.println(newPulseLen);
          }
        }
        flap.isOpen=true;
        flap.extended=0;
      }
      if(flap.isClosing) {
        stopMotion();
        if(flap.extended!=12 && autoPulseCalib) {
          float newPulseLen=12/(flap.extended/flap.ClosePulseLen);
          flap.ClosePulseLen=newPulseLen;
          if(debug) {
            Serial.print("New Close Pulse Length Calibration: ");
            Serial.println(newPulseLen);
          }
        }
        flap.isClosed=true;
        flap.extended=12;
      }
    }
  }
}

/*
   parseInstruction()
    canID flapTrigId ( 0x65 / 101 by default )
      byte 0:
        0 = Close
        1 = Open
        254 = Heartbeat
        255 = Stop
*/
void parseInstruction() {
  if (buf[0] == 254) {
    if(debug)
      Serial.println("Received control heartbeat");
    mystat.controller_received = millis();
    return;
  }
  if (buf[0] == 255) {
    if(debug)
      Serial.println("Received stop");
    stopMotion();
  }
  if (buf[0] == 0) {
    if(debug)
      Serial.println("Received close");
    closeFlap();
  }
  if (buf[0] == 1) {
    if(debug)
      Serial.println("Received open");
    openFlap();
  }
  sendState();
}

void stopMotion() {
  if(debug)
    Serial.println("Stopping Motion");
  digitalWrite(FLAP_ENABLE_PIN,0);
  digitalWrite(FLAP_DIRECTION_PIN,1);
  flap.desiredState=2;
  flap.isOpening=false;
  flap.isClosing=false;
}

void openFlap() {
  flap.desiredState=1;
  if(!flap.isOpening && !flap.isOpen) {
    digitalWrite(FLAP_ENABLE_PIN,0);
    flap.isClosing=false;
    flap.isClosed=false;
    digitalWrite(FLAP_DIRECTION_PIN,0);
    digitalWrite(FLAP_ENABLE_PIN,1);
    flap.isOpening=true;
    flap.lastPulse=millis(); // Fake, but needed to get the proper pulse readings
  }
}

void closeFlap() {
  flap.desiredState=0;
  if(!flap.isClosing && !flap.isClosed) {
    digitalWrite(FLAP_ENABLE_PIN,0);
    flap.isOpening=false;
    flap.isOpen=false;
    digitalWrite(FLAP_DIRECTION_PIN,1);
    digitalWrite(FLAP_ENABLE_PIN,1);
    flap.isClosing=true;
    flap.lastPulse=millis(); // Fake, but needed to get the proper pulse readings
  }
}
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
