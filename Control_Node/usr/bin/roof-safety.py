#!/usr/bin/env python3

import array, json, signal, subprocess, time
import paho.mqtt.client as mqtt

debug_level=0

def signal_handler(signal, frame):
    print(time.strftime("%Y-%m-%d %H:%M:%S") + ' You pressed Ctrl+C!  Exiting...', flush=True)
    exit(0)

signal.signal(signal.SIGINT, signal_handler)

last_status  = {}
issafe       = 0

states={ 'open':0, 'close':1, 'closed':1, 'opening':2, 'closing':3, 'err':4, 'heartbeat':254, 'stop':255 }

def log_output( msg, lvl ):
    global debug_level
    if lvl <= debug_level:
        print(time.strftime("%Y-%m-%d %H:%M:%S ") + msg, flush=True)

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    log_output( "Connected with result code " + str(rc), 1 )

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect, the subscriptions will be renewed.
    client.subscribe( [
        ("safety/issafe",0),
        ("roof/status",0),
        ("flap/status",0)
        ] )
    client.takingaction = 0
    client.eitherOpen = 0
    client.eitherClosing = 0


def on_message(client, userdata, msg):
    global issafe
    global s
    log_output( msg.topic + " " + str(msg.payload), 1 )

    dev=msg.topic.split( "/" )[ 0 ]
    if dev != "safety":
        try:
            last_status[ dev ] = json.loads( msg.payload.decode('ascii') )
            client.eitherClosing = 0
            client.eitherOpen = 0
            for mydev in last_status:
                if last_status[ mydev ][ 'state' ] == states['closing']:
                    client.eitherClosing = 1
                if last_status[ mydev ][ 'state' ] != states['closed']:
                    client.eitherOpen = 1
            if client.eitherOpen == 0:
                if client.takingaction > 0:
                    log_output( 'All closed now.', 0 )
                client.takingaction = 0

            if last_status[ dev ][ 'state' ] != states['closed'] and client.eitherClosing != 1:
                log_output( dev + " is not closed and neither device is closing...", 1 )
                if issafe != 1:
                    if time.time() - client.takingaction > 120:
                        client.takingaction = time.time()
                        log_output( "Unsafe! " + dev + " is not closed and neither device is closing, but was just informed it's not safe.  Closing roof now.", 0 ) 
                        stopProc  = subprocess.check_call( [ '/usr/bin/ascomroof.py', 'stop',  'both' ], stdout=subprocess.DEVNULL )
                        closeProc = subprocess.check_call( [ '/usr/bin/ascomroof.py', 'close', 'both' ], stdout=subprocess.DEVNULL )
                    else:
                        log_output( "Already took action", 1 )
        except Exception as e:
            log_output( 'Error loading json: ' + str(e) , 0 )
            exit(1)
    else:
        issafe=int( msg.payload.decode('ascii') )

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect( "127.0.0.1", 1883, 60 )
client.loop_forever()
