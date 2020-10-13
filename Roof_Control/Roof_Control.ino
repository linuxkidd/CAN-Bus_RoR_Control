/*
   Astro-Shed Roll-off-Roof Control
     by: Michael J. Kidd <linuxkidd@gmail.com>
    Rev: 2.0
   Date: 2020-10-11
*/

#include <SPI.h>

#include "mcp_can.h"

boolean debug                    = false;  // Enable debug output

// Uncomment the line below if you have a flap
// #define HAVE_FLAP

//const int CAN_SPEED            = CAN_250KBPS;    // For Uno  CAN-Bus Shield    (16MHz)
const int CAN_SPEED              = CAN_8M_250KBPS; // For nano CAN SPI interface  (8MHz)
const int SPI_CS_PIN             = 10;           // Which pin to use for SPI CS with CAN bus interface

const int ROOF_DIRECTION_PIN     = 9;
const int ROOF_ENABLE_PIN        = 8;

const int ROOF_OPEN_DETECT_PIN   = 7;
const int ROOF_CLOSED_DETECT_PIN = 5;

/*
 * Variables below this point are used internally and should not be changed.
 */
INT32U status_every               =  1000;  // Send status every 1 second ( 1000 ms )
INT32U controller_status_timeout  = 15000;  // If no controller heartbeat for 15 seconds, close the roof
#ifdef HAVE_FLAP
INT32U flap_status_timeout        = 15000;  // If no flap status for 15 seconds, close the roof
#endif 

unsigned char len = 0;
unsigned char buf[8];

unsigned int rxId       = 0x000;
unsigned int roofId     = 0x200;
unsigned int roofTrigId =  0x64;

#ifdef HAVE_FLAP
unsigned int flapId     = 0x300;
unsigned int flapTrigId =  0x65;
#endif 

void printPacket(unsigned int myrxId, unsigned int mylen, unsigned char mybuf[8]);
void parseInstruction();
void sendStartup();
void sendState();
void readPins();
void stopMotion();
void openRoof();
void closeRoof();

MCP_CAN CAN(SPI_CS_PIN);         // Set CS pin

typedef struct {
  INT32U transmitted = 0;
  INT32U controller_received = 0;
#ifdef HAVE_FLAP
  INT32U flap_received = 0;
#endif
} statustiming;

statustiming mystat;

typedef struct {
  boolean isOpen = false;
  boolean isClosed = false;
  boolean isOpening = false;
  boolean isClosing = false;
  unsigned int desiredState = 0; // 0 == closed, 1 == open, 2 == stopped
} roofstruct;

roofstruct roof;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Initialization");
  pinMode(ROOF_DIRECTION_PIN, OUTPUT);
  digitalWrite(ROOF_DIRECTION_PIN,1);
  pinMode(ROOF_ENABLE_PIN, OUTPUT);
  digitalWrite(ROOF_ENABLE_PIN,0);
  pinMode(ROOF_OPEN_DETECT_PIN, INPUT_PULLUP);
  pinMode(ROOF_CLOSED_DETECT_PIN, INPUT_PULLUP);
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

    if (rxId == roofTrigId)
      parseInstruction();

#ifdef HAVE_FLAP
    if (rxId == flapId)
      mystat.flap_received=millis();
#endif

    if(debug) {
      Serial.print("Received: ");
      printPacket( rxId, len,buf);
    }
  }

  readPins();
  if( mystat.controller_received + controller_status_timeout < millis() && !roof.isClosed && !roof.isClosing ) {
#ifdef HAVE_FLAP
    CAN.sendMsgBuf(flapTrigId,1,1,0); // Flap Close Command
#endif
    closeRoof();
  }

#ifdef HAVE_FLAP
  if( mystat.flap_received + flap_status_timeout < millis() && !roof.isClosed && !roof.isClosing ) {
    CAN.sendMsgBuf(flapTrigId,1,1,0); // Flap Close Command, just in case it's still alive
    closeRoof();
  }
#endif

  if( mystat.transmitted + status_every < millis() )
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
    printPacket(roofId+1,1,stmp);
  }
  CAN.sendMsgBuf(roofId+1,1,1,stmp);
}

void sendState() {
  unsigned char stmp[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
  if(roof.isOpen)
    stmp[0]=1;
  if(roof.isClosed)
    stmp[1]=1;
  if(roof.isOpening)
    stmp[2]=1;
  if(roof.isClosing)
    stmp[3]=1;
  if(roof.desiredState)
    stmp[4]=roof.desiredState;
  if(debug) {
    Serial.write("Sending State: ");
    printPacket(roofId,5,stmp);
  }
  CAN.sendMsgBuf(roofId,1,5,stmp);
  mystat.transmitted=millis();
}

void readPins() {
  if(digitalRead(ROOF_OPEN_DETECT_PIN) == LOW) {
    if(roof.isOpening)
      stopMotion();
    roof.isOpen=true;
  } else {
    roof.isOpen=false;
    if(roof.desiredState==1 && roof.isOpening==0)
      openRoof();
  }

  if(digitalRead(ROOF_CLOSED_DETECT_PIN) == LOW) {
    if(roof.isClosing)
      stopMotion();
    roof.isClosed=true;
  } else {
    roof.isClosed=false;
    if(roof.desiredState==0 && roof.isClosing==0)
      closeRoof();
  }
}

/*
   parseInstruction()
    canID roofTrigId ( 0x64 / 100 by default )
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
    mystat.controller_received=millis();
    return;
  }
  if (buf[0] == 255) {
    stopMotion();
    if(debug)
      Serial.println("Received stop");
  }
  if (buf[0] == 0) {
    if(debug)
      Serial.println("Received close");
    closeRoof();
  }
  if (buf[0] == 1) {
    if(debug)
      Serial.println("Received open");
    openRoof();
  }
  sendState();
}

void stopMotion() {
  if(debug)
    Serial.println("Stopping Motion");
  digitalWrite(ROOF_ENABLE_PIN,0);
  digitalWrite(ROOF_DIRECTION_PIN,1);
  roof.desiredState=2;
  roof.isOpening=false;
  roof.isClosing=false;
}

void openRoof() {
  roof.desiredState=1;
  if(!roof.isOpening && !roof.isOpen) {
    digitalWrite(ROOF_ENABLE_PIN,0);
    roof.isClosing=false;
    digitalWrite(ROOF_DIRECTION_PIN,0);
    digitalWrite(ROOF_ENABLE_PIN,1);
    roof.isOpening=true;
  }
  delay(50);
}

void closeRoof() {
  roof.desiredState=0;
  if(!roof.isClosing && !roof.isClosed) {
    digitalWrite(ROOF_ENABLE_PIN,0);
    roof.isOpening=false;
    digitalWrite(ROOF_DIRECTION_PIN,1);
    digitalWrite(ROOF_ENABLE_PIN,1);
    roof.isClosing=true;
  }
  delay(50);
}
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
