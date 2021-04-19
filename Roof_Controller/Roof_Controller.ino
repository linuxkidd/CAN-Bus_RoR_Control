/*
   Astro-Shed Roll-off-Roof Control
     by: Michael J. Kidd <linuxkidd@gmail.com>
     at: https://github.com/linuxkidd/CAN-Bus_RoR_Control
    Rev: 3.0
   Date: 2020-10-14
*/

#include <SPI.h>

#include "mcp_can.h"

boolean debug                    = true;  // Enable debug output

// Uncomment either ENABLE_PLUS_DIRECTION
//   or OPEN_CLOSE
// below to indicate how the relays should be controlled
// #define ENABLE_PLUS_DIRECTION
#define OPEN_CLOSE
// #define ENABLE_PLUS_DIRECTION

// Uncomment the line below if you have a flap
// #define HAVE_FLAP

// Uncomment the line below if you have Pier Safety sensor
#define HAVE_PIER_SAFETY

#define CAN_SPEED      (CAN_250KBPS)     // For Uno  CAN-Bus Shield    (16MHz)
//#define CAN_SPEED   (CAN_8M_250KBPS)     // For nano CAN SPI interface  (8MHz)
#define SPI_CS_PIN              (10)     // Which pin to use for SPI CS with CAN bus interface

#ifdef ENABLE_PLUS_DIRECTION
#define ROOF_DIRECTION_PIN       (9)
#define ROOF_ENABLE_PIN          (8)
#endif

#ifdef OPEN_CLOSE
#define ROOF_OPEN_PIN            (9)
#define ROOF_CLOSE_PIN           (8)
#endif

#define ROOF_OPEN_DETECT_PIN     (7)
#define ROOF_CLOSED_DETECT_PIN   (6)

#ifdef HAVE_PIER_SAFETY
#define PIER_SAFE_PIN            (3)
#endif

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

#ifdef HAVE_PIER_SAFETY
bool piersafety_laststate = false;
INT32U piersafety_statetime = 0;
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

// These values follow ASCOM Alpaca 'shutterstatus' states, with the addition of heartbeat and stop
#define OPEN        (0)
#define CLOSED      (1)
#define OPENING     (2)
#define CLOSING     (3)
#define ERR         (4)
#define HEARTBEAT (254)
#define STOP      (255)
#ifdef HAVE_PIER_SAFETY
// Safety Override movement commands
#define OROPEN    (128)
#define ORCLOSE   (129)
#endif

typedef struct {
  unsigned int state = CLOSED;
  unsigned int desiredState = ERR;
  bool isOpen = false;
  bool isClosed = false;
  bool isSafe = true;
  bool overrideSafe = false;
} roofstruct;

roofstruct roof;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Initialization");
#ifdef ENABLE_PLUS_DIRECTION
  pinMode(ROOF_DIRECTION_PIN, OUTPUT);
  digitalWrite(ROOF_DIRECTION_PIN,1);
  pinMode(ROOF_ENABLE_PIN, OUTPUT);
  digitalWrite(ROOF_ENABLE_PIN,0);
#endif

#ifdef OPEN_CLOSE
  pinMode(ROOF_OPEN_PIN, OUTPUT);
  digitalWrite(ROOF_OPEN_PIN,1);
  pinMode(ROOF_CLOSE_PIN, OUTPUT);
  digitalWrite(ROOF_CLOSE_PIN,1);
#endif
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
  if( mystat.controller_received + STATUS_TIMEOUT < millis() 
    && roof.state!=CLOSED 
    && roof.state!=CLOSING 
    && roof.desiredState!=CLOSED 
    && roof.desiredState!=STOP) {
    if(debug)
      Serial.println("Heartbeat timeout - Closing.");

#ifdef HAVE_FLAP
    unsigned char closed[1] = { CLOSED };
    CAN.sendMsgBuf(FLAP_COMMAND_ID,1,1,closed); // Flap Close Command
#endif
    closeRoof();
  }

#ifdef HAVE_FLAP
  if( mystat.flap_received + STATUS_TIMEOUT < millis()
    && roof.state!=CLOSED
    && roof.state!=CLOSING
    && roof.desiredState!=CLOSED
    && roof.desiredState!=STOP ) {
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
#ifdef HAVE_PIER_SAFETY
  unsigned char stmp[5] = { (unsigned char)roof.state,  (unsigned char)roof.desiredState, 
                            (unsigned char)roof.isOpen, (unsigned char)roof.isClosed,
                            (unsigned char)roof.isSafe                                    };
#else
  unsigned char stmp[4] = { (unsigned char)roof.state,  (unsigned char)roof.desiredState, 
                            (unsigned char)roof.isOpen, (unsigned char)roof.isClosed      };
#endif

  if(debug) {
    Serial.write("Last Controller Received: ");
    Serial.println(mystat.controller_received);
    Serial.write("Sending State: ");
#ifdef HAVE_PIER_SAFETY
    printPacket(ROOF_ID,5,stmp);
#else
    printPacket(ROOF_ID,4,stmp);
#endif
  }
#ifdef HAVE_PIER_SAFETY
  CAN.sendMsgBuf(ROOF_ID,1,5,stmp);
#else
  CAN.sendMsgBuf(ROOF_ID,1,4,stmp);
#endif
  mystat.transmitted=millis();
}

void readPins() {
#ifdef HAVE_PIER_SAFETY
  if(digitalRead(PIER_SAFE_PIN) == LOW) {
    if(piersafety_laststate == true) {
      piersafety_laststate = false;
      piersafety_statetime = millis();
    }
    if(!roof.isSafe && piersafety_laststate == false && piersafety_statetime > 0 && piersafety_statetime + 1000 < millis()) {
      roof.isSafe=1;
      if ( debug )
        Serial.println("Pier now safe.");
    }
  } else {
    if(piersafety_laststate == false) {
      piersafety_laststate = true;
      piersafety_statetime = millis();
    }
    if(roof.isSafe && piersafety_laststate == true && piersafety_statetime > 0 && piersafety_statetime + 1000 < millis()) {
      roof.isSafe=0;
      if(roof.state==OPENING || roof.state==CLOSING) {
        stopMotion();
        roof.state=ERR;
      }
      if ( debug )
        Serial.println("Pier NOT safe.");
    }
  }
#endif
  if(digitalRead(ROOF_OPEN_DETECT_PIN) == LOW) {
    if(!roof.isOpen || roof.state!=OPEN) {
      if(debug)
        Serial.println("Roof is OPEN.");
      roof.isOpen=true;
      if(roof.state==OPENING || roof.state==ERR) {
        stopMotion();
        roof.state=OPEN;
      }
#ifdef HAVE_PIER_SAFETY
      roof.overrideSafe=0;
#endif
    }
  } else {
    if(roof.isOpen || roof.state==OPEN) {
      if(debug)
        Serial.println("Roof not OPEN.");
      roof.isOpen=false;
      if(roof.state!=CLOSING)
        roof.state=ERR;
    }
    if(roof.desiredState==OPEN && roof.state!=OPENING)
      openRoof();
  }

  if(digitalRead(ROOF_CLOSED_DETECT_PIN) == LOW) {
    if(!roof.isClosed || roof.state!=CLOSED) {
      if(debug)
        Serial.println("Roof is CLOSED.");
      roof.isClosed=true;
      if(roof.state==CLOSING || roof.state==ERR) {
        // Only change roof.state to CLOSED if it was attempting to close.
        // Otherwise, if the roof just started to open, the closed pin may still 
        // be low, giving a false 'CLOSED' status, when the status should be 'OPENING'
        roof.state=CLOSED;
        stopMotion();
      }
#ifdef HAVE_PIER_SAFETY
      roof.overrideSafe=0;
#endif
    }
  } else {
    if(roof.isClosed || roof.state==CLOSED) {
      if(debug)
        Serial.println("Roof not CLOSED.");
      roof.isClosed=false;
      if(roof.state!=OPENING)
        roof.state=ERR;
    }
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
#ifdef HAVE_PIER_SAFETY
  else if (buf[0] == OROPEN) {
    if ( debug )
      Serial.println("Received OVERRIDE open.");
    roof.overrideSafe=1;
    openRoof();
  } else if (buf[0] == ORCLOSE) {
    if ( debug )
      Serial.println("Received OVERRIDE close.");
    roof.overrideSafe=1;
    closeRoof();
  }
#endif
  sendState();
}

#ifdef ENABLE_PLUS_DIRECTION
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
#ifdef HAVE_PIER_SAFETY
  if(!roof.isSafe && !roof.overrideSafe) {
#endif
  if(roof.state!=OPENING && !roof.isOpen) {
    digitalWrite(ROOF_ENABLE_PIN,0);

    delay(100); // Allow any prior motion to stop

    digitalWrite(ROOF_DIRECTION_PIN,0);
    digitalWrite(ROOF_ENABLE_PIN,1);
    roof.state=OPENING;
  }
  delay(50);
#ifdef HAVE_PIER_SAFETY
  }
#endif
}

void closeRoof() {
  roof.desiredState=CLOSED;
#ifdef HAVE_PIER_SAFETY
  if(roof.isSafe || roof.overrideSafe) {
#endif
  if(roof.state!=CLOSING && !roof.isClosed) {
    digitalWrite(ROOF_ENABLE_PIN,0);
    delay(100);
    digitalWrite(ROOF_DIRECTION_PIN,1);
    digitalWrite(ROOF_ENABLE_PIN,1);
    roof.state=CLOSING;
  }
  delay(50);
#ifdef HAVE_PIER_SAFETY
  }
#endif
}
#endif // ENABLE_PLUS_DIRECTION

#ifdef OPEN_CLOSE
void stopMotion() {
  if(debug)
    Serial.println("Stopping Motion");
  digitalWrite(ROOF_OPEN_PIN,1);
  digitalWrite(ROOF_CLOSE_PIN,1);
  delay(50);
  // Not setting any roof.state or roof.desiredState here.
  // That status must be set appropriately in calling function.
}

void openRoof() {
  roof.desiredState=OPEN;
#ifdef HAVE_PIER_SAFETY
  if(roof.isSafe || roof.overrideSafe) {
#endif
  if(roof.state!=OPENING && !roof.isOpen) {
    digitalWrite(ROOF_CLOSE_PIN,1);
    delay(100); // Allow any prior motion to stop

    digitalWrite(ROOF_OPEN_PIN,1);
    roof.state=OPENING;
  }
  delay(50);
#ifdef HAVE_PIER_SAFETY
  }
#endif
}

void closeRoof() {
  roof.desiredState=CLOSED;
#ifdef HAVE_PIER_SAFETY
  if(roof.isSafe || roof.overrideSafe) {
#endif
  if(roof.state!=CLOSING && !roof.isClosed) {
    digitalWrite(ROOF_OPEN_PIN,1);
    delay(100); // Allow any prior motion to stop

    digitalWrite(ROOF_CLOSE_PIN,0);
    roof.state=CLOSING;
  }
  delay(50);
#ifdef HAVE_PIER_SAFETY
  }
#endif
}
#endif // OPEN_CLOSE
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
