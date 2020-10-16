/*
   Astro-Shed Roll-off-Roof Control
     by: Michael J. Kidd <linuxkidd@gmail.com>
     at: https://github.com/linuxkidd/CAN-Bus_RoR_Control
    Rev: 3.0
   Date: 2020-10-14
*/

#include <SPI.h>

#include "mcp_can.h"

boolean debug                    = false;  // Enable debug output

// Uncomment the line below if you have a flap
// #define HAVE_FLAP

//#define CAN_SPEED    (CAN_250KBPS)     // For Uno  CAN-Bus Shield    (16MHz)
#define CAN_SPEED   (CAN_8M_250KBPS)     // For nano CAN SPI interface  (8MHz)
#define SPI_CS_PIN              (10)     // Which pin to use for SPI CS with CAN bus interface

#define ROOF_DIRECTION_PIN       (9)
#define ROOF_ENABLE_PIN          (8)

#define ROOF_OPEN_DETECT_PIN     (7)
#define ROOF_CLOSED_DETECT_PIN   (5)

/*
 * If no heartbeat from control or roof nodes for 15 seconds, close the roof
 */
#define STATUS_TIMEOUT        (15000)

/*
 * Send status every 1 second ( 1000 ms )
 * 
 */

#define STATUS_EVERY           (1000)

/*
 * Pre-defined CAN IDs 
 * 
 * ## WARNING ##
 * -- don't change these unless you know what you're doing.
 * 
 */

#define ROOF_ID               (0x200)
/*
 * NOTE: 
 *   ROOF_ID+1 used for boot notification
 */
#define ROOF_COMMAND_ID        (0x64)

#ifdef HAVE_FLAP
#define FLAP_ID               (0x300)
/*
 * NOTE: 
 *   FLAP_ID+1 used for boot notification
 */

#define FLAP_COMMAND_ID        (0x65)
#endif 


/*
 * Variables below this point are for status tracking and should not be changed.
 * 
 */
unsigned char len     =     0;
unsigned char buf[8];
unsigned int  rxId    = 0x000;

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

// These values follow ASCOM Alpaca 'shutterstatus' states, with the addition of heartbeat and stop
#define OPEN        (0)
#define CLOSED      (1)
#define OPENING     (2)
#define CLOSING     (3)
#define ERR         (4)
#define HEARTBEAT (254)
#define STOP      (255)

typedef struct {
  unsigned int state = CLOSED;
  unsigned int desiredState = STOP;
  bool isOpen = false;
  bool isClosed = false;
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

    if (rxId == ROOF_COMMAND_ID)
      parseInstruction();

#ifdef HAVE_FLAP
    if (rxId == FLAP_ID)
      mystat.flap_received=millis();
#endif

    if(debug) {
      Serial.print("Received: ");
      printPacket( rxId, len,buf);
    }
  }

  readPins();
  if( mystat.controller_received + STATUS_TIMEOUT < millis() && roof.state!=CLOSED && roof.state !=CLOSING ) {
#ifdef HAVE_FLAP
    unsigned char closed[1] = { CLOSED };
    CAN.sendMsgBuf(FLAP_COMMAND_ID,1,1,closed); // Flap Close Command
#endif
    closeRoof();
  }

#ifdef HAVE_FLAP
  if( mystat.flap_received + STATUS_TIMEOUT < millis() && roof.state!=CLOSED && roof.state!=CLOSING ) {
    unsigned char closed[1] = { CLOSED };
    CAN.sendMsgBuf(FLAP_COMMAND_ID,1,1,closed); // Flap Close Command, just in case it's still alive
    closeRoof();
  }
#endif

  if( mystat.transmitted + STATUS_EVERY < millis() )
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
    printPacket(ROOF_ID+1,1,stmp);
  }
  CAN.sendMsgBuf(ROOF_ID+1,1,1,stmp);
}

void sendState() {
  /* 
   * Send ASCOM Alpaca 'shutterstatus' compatible state and desiredState 
   * along with actual limit switch positions
   */
  unsigned char stmp[4] = { (unsigned char)roof.state,  (unsigned char)roof.desiredState, 
                            (unsigned char)roof.isOpen, (unsigned char)roof.isClosed      };

  if(debug) {
    Serial.write("Sending State: ");
    printPacket(ROOF_ID,4,stmp);
  }
  CAN.sendMsgBuf(ROOF_ID,1,4,stmp);
  mystat.transmitted=millis();
}

void readPins() {
  if(digitalRead(ROOF_OPEN_DETECT_PIN) == LOW) {
    roof.isOpen=true;
    if(roof.state==OPENING) {
      stopMotion();
      roof.state=OPEN;
    }
  } else {
    roof.isOpen=false;
    if(roof.desiredState==OPEN && roof.state!=OPENING)
      openRoof();
  }

  if(digitalRead(ROOF_CLOSED_DETECT_PIN) == LOW) {
    roof.isClosed=true;
    if(roof.state==CLOSING) {
      // Only change roof.state to CLOSED if it was attempting to close.
      // Otherwise, if the roof just started to open, the closed pin may still 
      // be low, giving a false 'CLOSED' status, when the status should be 'OPENING'
      roof.state=CLOSED;
      stopMotion();
    }
  } else {
    roof.isClosed=false;
    if(roof.desiredState==CLOSED && roof.state!=CLOSING)
      closeRoof();
  }
}

/*
   parseInstruction()
    canID ROOF_COMMAND_ID ( 0x64 / 100 by default )
*/
void parseInstruction() {
  if (buf[0] == HEARTBEAT) {
    if(debug)
      Serial.println("Received control heartbeat");
    mystat.controller_received=millis();
    return;
  } else if (buf[0] == STOP) {
    stopMotion();
    roof.desiredState=STOP;
    roof.state=ERR;
    if(debug)
      Serial.println("Received stop");
  } else if (buf[0] == CLOSED) {
    if(debug)
      Serial.println("Received close");
    closeRoof();
  } else if (buf[0] == OPEN) {
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
  delay(50);
  // Not setting any roof.state or roof.desiredState here.
  // That status must be set appropriately in calling function.
}

void openRoof() {
  roof.desiredState=OPEN;
  if(roof.state!=OPENING && !roof.isOpen) {
    digitalWrite(ROOF_ENABLE_PIN,0);

    delay(100); // Allow any prior motion to stop

    digitalWrite(ROOF_DIRECTION_PIN,0);
    digitalWrite(ROOF_ENABLE_PIN,1);
    roof.state=OPENING;
  }
  delay(50);
}

void closeRoof() {
  roof.desiredState=CLOSED;
  if(roof.state!=CLOSING && !roof.isClosed) {
    digitalWrite(ROOF_ENABLE_PIN,0);
    delay(100);
    digitalWrite(ROOF_DIRECTION_PIN,1);
    digitalWrite(ROOF_ENABLE_PIN,1);
    roof.state=CLOSING;
  }
  delay(50);
}
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
