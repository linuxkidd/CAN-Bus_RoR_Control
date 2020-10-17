/*
   Astro-Shed Flap Control
     by: Michael J. Kidd <linuxkidd@gmail.com>
     at: https://github.com/linuxkidd/CAN-Bus_RoR_Control
    Rev: 3.0
   Date: 2020-10-14
*/

#include <SPI.h>

#include "mcp_can.h"

boolean debug               = false;     // Enable debug output

//#define CAN_SPEED    (CAN_250KBPS)     // For Uno  CAN-Bus Shield    (16MHz)
#define CAN_SPEED   (CAN_8M_250KBPS)     // For nano CAN SPI interface  (8MHz)
#define SPI_CS_PIN              (10)     // Which pin to use for SPI CS with CAN bus interface

#define FLAP_DIRECTION_PIN       (9)
#define FLAP_ENABLE_PIN          (8)

#define FLAP_MOVE_DETECT_PIN     (7)

/*
 * I'm using actuator https://www.amazon.com/gp/product/B0713YWQQ4
 * Set 'FLAP_MAX_EXTEND' to the extension length, in my case, 12"
 */

#define FLAP_MAX_EXTEND          (12)

/*
 *  Specs state the 'FEEDBACK' output from Hall Effect sensor provides a
 *  pulsed output, with each pulse accounting for 0.007046" of travel.
 *  -- Note that the code below counts each transition, both rising and
 *     falling.  Thus, the distance per pulse is divided by 2 since each
 *     "pulse" is counted twice ( rising and falling ).
 */

#define PULSE_LEN        (0.007046/2)

/*
 * How many milliseconds of no pulses should be used to determine motion has stopped?
 *  -- 500 seems to be more than sufficient.
 */

#define PULSE_TIMEOUT           (500)

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

#define ROOF_ID           (0x200)
#define FLAP_ID           (0x300)
#define ROOF_COMMAND_ID    (0x64)
#define FLAP_COMMAND_ID    (0x65)


/*
 * Variables below this point are for status tracking and should not be changed.
 * 
 */
unsigned char len     =     0;
unsigned char buf[8];
unsigned int  rxId     = 0x000;


bool lastMovePinState =   LOW;
bool MovePinState     =   LOW;

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


// These values follow ASCOM Alpaca 'shutterstatus' states, with the addition of heartbeat and stop
#define OPEN        (0)
#define CLOSED      (1)
#define OPENING     (2)
#define CLOSING     (3)
#define ERR         (4)
#define HEARTBEAT (254)
#define STOP      (255)


typedef struct {
  unsigned int state        =          CLOSED;
  unsigned int desiredState =          CLOSED;
  INT32U lastPulse          =               0;
  float extended            = FLAP_MAX_EXTEND;
  bool lastPinState         =             LOW;
  bool PinState             =             LOW;
} flapstruct;

flapstruct flap;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Initialization");
  pinMode(      FLAP_DIRECTION_PIN,         OUTPUT );
  digitalWrite( FLAP_DIRECTION_PIN,              1 );

  pinMode(      FLAP_ENABLE_PIN,            OUTPUT );
  digitalWrite( FLAP_ENABLE_PIN,                 0 );

  pinMode(      FLAP_MOVE_DETECT_PIN, INPUT_PULLUP );

  Serial.println("PIN Config complete");

START_INIT:
  Serial.println("Start CAN config");
  if ( CAN_OK == CAN.begin( CAN_SPEED ) )  {
    CAN.init_Mask( 0, 1, 0 );
    CAN.init_Mask( 1, 1, 0 );
    for ( int i = 0; i < 6; i++ ) {
      CAN.init_Filt( i, 1, 0 );
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

    if( rxId == FLAP_COMMAND_ID )
      parseInstruction();

    if( rxId == ROOF_ID )
      mystat.roof_received=millis();

    if( debug ) {
      Serial.print("Received: ");
      printPacket(rxId,len,buf);
    }
  }

  readPins();
  if( mystat.controller_received + STATUS_TIMEOUT < millis() && flap.state!=CLOSED && flap.state!=CLOSING ) {
    unsigned char closed[1] = { CLOSED };
    CAN.sendMsgBuf(ROOF_COMMAND_ID,1,1,closed); // Roof Close Command
    closeFlap();
  }

  if( mystat.roof_received + STATUS_TIMEOUT < millis() && flap.state!=CLOSED && flap.state!=CLOSING ) {
    unsigned char closed[1] = { CLOSED };
    CAN.sendMsgBuf(ROOF_COMMAND_ID,1,1,closed); // Roof Close Command, just in case it's still listening
    closeFlap();
  }

  if( mystat.transmitted + STATUS_EVERY < millis() )
    sendState();
}

void printPacket( unsigned int myrxId, unsigned int mylen, unsigned char mybuf[8] ) {
  Serial.print(millis());
  Serial.print("\t\t");
  Serial.print("0x");
  Serial.print(myrxId, HEX);
  Serial.print("\t\t");

  for( int i = 0; i<mylen; i++ ) {   // print the data
    if( mybuf[i] > 15 ) {
      Serial.print("0x");
      Serial.print(mybuf[i], HEX);    
    } else {
      Serial.print("0x0");
      Serial.print(mybuf[i], HEX);
    }      
    Serial.print("\t");            
  }
  Serial.println();
}

void sendStartup() {
  unsigned char stmp[1] = {0xff};
  if( debug ) {
    Serial.write("Sending State: ");
    printPacket( FLAP_ID+1, 1, stmp );
  }
  CAN.sendMsgBuf( FLAP_ID+1, 1, 1, stmp);
}

void sendState() {
  int mypos = flap.extended * ( 255.0 / FLAP_MAX_EXTEND ); // Calculate position in 256 steps
  unsigned char stmp[3] = { (unsigned char)flap.state, (unsigned char)flap.desiredState, (unsigned char)mypos };
  if( debug ) {
    Serial.write("Sending State: ");
    printPacket( FLAP_ID, 3, stmp );
  }
  CAN.sendMsgBuf( FLAP_ID, 1, 3, stmp );
  mystat.transmitted = millis();
}

void readPins() {
  flap.PinState = digitalRead( FLAP_MOVE_DETECT_PIN );
  if( flap.PinState != flap.lastPinState ) {
    flap.lastPinState = flap.PinState;
    flap.lastPulse = millis();
    if( flap.state == OPENING )
      flap.extended -= PULSE_LEN;
    else
      flap.extended += PULSE_LEN;

    if( flap.extended > FLAP_MAX_EXTEND )
      flap.extended = FLAP_MAX_EXTEND;
    if( flap.extended < 0 )
      flap.extended = 0;

    if( debug ) {
      Serial.print("Pulse: ");
      Serial.print(flap.lastPulse);
      Serial.print(", ");
      
      Serial.print(flap.extended);
      Serial.print(", State: ");
      Serial.print(flap.state);
      Serial.print(", Desired State: ");
      Serial.print(flap.desiredState);
      Serial.println();
    }

  } else {
    if( flap.lastPulse + PULSE_TIMEOUT < millis() ) {
      if( debug && ( flap.state == OPENING || flap.state == CLOSING ) )
        Serial.print("Was Opening/Closing, but no pulses in ");
        Serial.print(PULSE_TIMEOUT);
        Serial.println(" ms.");
      if( flap.state == OPENING ) {
        stopMotion();
        flap.state = OPEN;
        flap.extended = 0;
      }
      if( flap.state == CLOSING ) {
        stopMotion();
        flap.state = CLOSED;
        flap.extended = FLAP_MAX_EXTEND;
      }
    }
  }
}

/*
 *  parseInstruction()
 *   canID FLAP_COMMAND_ID ( 0x65 / 101 by default )
 */
void parseInstruction() {
  if ( buf[0] == HEARTBEAT ) {
    if( debug )
      Serial.println("Received control heartbeat");
    mystat.controller_received = millis();
    return;
  } else if ( buf[0] == STOP ) {
    stopMotion();
    flap.desiredState = STOP;
    flap.state = ERR;
    if( debug )
      Serial.println("Received stop");
  } else if ( buf[0] == CLOSED ) {
    if( debug )
      Serial.println("Received close");
    closeFlap();
  } else if ( buf[0] == OPEN ) {
    if( debug )
      Serial.println("Received open");
    openFlap();
  }
  sendState();
}

void stopMotion() {
  if( debug )
    Serial.println("Stopping Motion");
  digitalWrite( FLAP_ENABLE_PIN,    0 );
  digitalWrite( FLAP_DIRECTION_PIN, 1 );
  // Not setting any roof.state or roof.desiredState here.
  // That status must be set appropriately in calling function.
}

void openFlap() {
  flap.desiredState = OPEN;
  if( flap.state != OPENING && flap.state != OPEN ) {
    digitalWrite( FLAP_ENABLE_PIN, 0);

    delay(100); // Allow any prior motion to stop

    digitalWrite( FLAP_DIRECTION_PIN, 0 );
    digitalWrite( FLAP_ENABLE_PIN, 1 );
    flap.state = OPENING;
    flap.lastPulse = millis(); // Fake, but needed to get the proper pulse readings
  }
}

void closeFlap() {
  flap.desiredState = CLOSED;
  if( flap.state != CLOSING && flap.state != CLOSED ) {
    digitalWrite( FLAP_ENABLE_PIN, 0 );

    delay(100); // Allow any prior motion to stop

    digitalWrite( FLAP_DIRECTION_PIN, 1 );
    digitalWrite( FLAP_ENABLE_PIN, 1 );
    flap.state = CLOSING;
    flap.lastPulse = millis(); // Fake, but needed to get the proper pulse readings
  }
}
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
