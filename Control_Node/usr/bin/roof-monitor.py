#!/usr/bin/env python3

import argparse,array,can,json,os,queue,re,signal,time
from threading import Thread

flap_max_extend = 12

device_command_ids = { 'roof':0x64,'flap':0x65 }
device_commands = { 'open': 0x00, 'close': 0x01, 'opening': 0x02, 'closing': 0x03, 'error': 0x04, 'heartbeat':0xfe, 'stop':0xff }

incoming_status_ids = { 0x200:"roof", 0x201:"roofReboot", 0x300:"flap", 0x301:"flapReboot" }
incoming_status_fields = {
        "roof":       [ 'state', 'desiredState', 'isOpen', 'isClosed' ],
        "roofReboot": [ 'count' ],
        "flap":       [ 'state', 'desiredState', 'extended' ],
        "flapReboot": [ 'count' ]
        }

last_incoming_status = { 'roof': {}, 'flap': {}, 'roofReboot': {}, 'flapReboot': {} }
last_heartbeat_transmitted = 0

def signal_handler( signal, frame ):
    print('')
    print('You pressed Ctrl+C!  Exiting...')
    print('')
    exit(0)

signal.signal( signal.SIGINT, signal_handler )

def on_mqtt_connect( client, userdata, flags, rc ):
    if debug_level:
        print("MQTT Connected with code " + str(rc))
    client.subscribe( [
        ( "flap/command", 0 ),
        ( "roof/command", 0 ) 
        ] )

def on_mqtt_subscribe( client, userdata, mid, granted_qos ):
    if debug_level:
        print("MQTT Sub: "+str(mid))

def on_mqtt_message( client, userdata, msg ):
    dev=msg.topic.split("/")[0]
    #if debug_level:
    print( "Send {0:s} Command: {1:s}".format( dev, msg.payload.decode('ascii') ) )
    print( "Sending: {0:02x} {1:02x}".format( device_command_ids[dev], device_commands[ msg.payload.decode('ascii') ] ) )
    can_tx( device_command_ids[dev], [ device_commands[ msg.payload.decode('ascii') ] ] )

def can_tx( canid, canmsg ):
    msg = can.Message( arbitration_id=canid, data=canmsg, extended_id=True )
    try:
        bus.send( msg )
        if debug_level > 0:
            print("Message sent on {}".format( bus.channel_info ))
    except can.CanError:
        print("CAN Send Failed")

def can_rx_task():
    while True:
        message = bus.recv()
        q.put(message)  # Put message into queue

def can_tx_heartbeat():
    global last_heartbeat_transmitted
    for i in [ 'roof', 'flap']:
        if debug_level:
            print("Sending {0:s} heartbeat at {1:d}".format( i, time.time() ) )
        can_tx( device_command_ids[i], [ device_commands['heartbeat'] ] )
    last_heartbeat_transmitted = int( time.time() )

def main():
    global last_heartbeat_transmitted
    retain = False
    can_tx_heartbeat()
    if( mqttOut == 2 ):
        retain = True

    def getLine():
        if q.empty():  # Check if there is a message in queue
            return
    
        message = q.get()
        if debug_level > 0:
            print("{0:f} {1:d} ({2:d}) ".format( message.timestamp, message.arbitration_id, message.dlc), end='' )
        try:
            dev = incoming_status_ids[ message.arbitration_id ]
        except:
            print("Failed to find Device for CanID: {0:d}".format( message.arbitration_id ) )
            return
        if debug_level > 0:
            print("Device: {0:s}: ".format( dev ), end='', flush=True )
        for i in range( message.dlc ):
            field = incoming_status_fields[ dev ][ i ]
            if debug_level > 0:
                print("{0:s} = {1:02x}, ".format( field, message.data[i] ), end='', flush=True)
            if field == 'count':
                try:
                    last_incoming_status[ dev ][ field ] += 1
                except:
                    last_incoming_status[ dev ][ field ] = 1
            elif field == 'extended':
                last_incoming_status[ dev ][ field ] = round( message.data[ i ] / ( 255 / flap_max_extend ) * 100 ) / 100
            else:
                last_incoming_status[ dev ][ field ] = message.data[ i ]
            if mqttOut:
                mqttc.publish("{0:s}/{1:s}".format( dev, field ), str( last_incoming_status[ dev ][ field ] ) + "," + str( message.timestamp ), retain = retain )
        last_incoming_status[ dev ][ "pktTime" ] = message.timestamp
        if debug_level > 0:
            print("")
            print("{0:s}: ".format( dev ), end='')
            print(json.dumps( last_incoming_status[ dev ] ))
        if mqttOut and field != 'count':
            mqttc.publish( dev + "/status", json.dumps( last_incoming_status[ dev ] ) )

    def mainLoop():
        if mqttOut:
            mqttc.loop_start()
        while True:
            getLine()
            if int( time.time() ) - last_heartbeat_transmitted == 1:
                can_tx_heartbeat()
            time.sleep(0.05)
        if mqttOut:
            mqttc.loop_stop()
    mainLoop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--interface", default = "can0",                           help="CAN interface to use")
    parser.add_argument("-d", "--debug",     default = 0, type = int, choices=[0, 1, 2], help="debug data")
    parser.add_argument("-m", "--mqtt",      default = 0, type = int, choices=[0, 1, 2], help="Send to MQTT, 0 = Disabled, 1 = Publish, 2 = Retain")
    parser.add_argument("-b", "--broker",    default = '127.0.0.1',                      help="MQTT Broker, default 127.0.0.1")
    args = parser.parse_args()

    debug_level = args.debug 
    mqttOut = args.mqtt

    if mqttOut:
        import paho.mqtt.client as mqtt
        mqttc = mqtt.Client() #create new instance
        mqttc.on_connect = on_mqtt_connect
        mqttc.on_subscribe = on_mqtt_subscribe
        mqttc.on_message = on_mqtt_message

        try:
            mqttc.connect(args.broker, port=1883) #connect to broker
        except:
            print("MQTT Broker ( {0:s} ) Connection Failed".format(args.broker))

    try:
        bus = can.interface.Bus( channel = args.interface, bustype = 'socketcan_native' )
    except OSError:
        print('Cannot find SocketCAN device.')
        exit()

    q = queue.Queue()	
    t = Thread( target = can_rx_task )  # Start receive thread
    t.start()


    main()
