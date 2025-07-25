#!/usr/bin/env python3

import argparse,array,json,os,queue,re,serial,signal,time
from threading import Thread

devIds={'roof':0x64}
commands={'open': 0x00, 'close': 0x01, 'opening': 0x02, 'closing': 0x03, 'error': 0x04, 'oropen': 0x80, 'orclose': 0x81, 'heartbeat':0xfe, 'stop':0xff}
incomingIds={0x200:"roof",0x201:"roofReboot"}
statusPosition={
        "roof":['state','desiredState','isOpen','isClosed', 'isSafe'],
        "roofReboot":['count']
        }

lastStatus={'roof':{}, 'roofReboot':{}}
lastHeartbeat=0
ser=""

def signal_handler(signal, frame):
    print('')
    print('You pressed Ctrl+C!  Exiting...')
    print('')
    exit(0)

signal.signal(signal.SIGINT, signal_handler)

def on_mqtt_connect(client, userdata, flags, rc):
    if debug_level:
        print("MQTT Connected with code "+str(rc))
    client.subscribe([("roof/command",0)])

def on_mqtt_subscribe(client, userdata, mid, granted_qos):
    if debug_level:
        print("MQTT Sub: "+str(mid))

def on_mqtt_message(client, userdata, msg):
    dev=msg.topic.split("/")[0]
    if debug_level:
        print("Command: "+msg.payload.decode('ascii'))
        print("Sending: {0:02x}".format(commands[msg.payload.decode('ascii')]))
    serial_tx(commands[msg.payload.decode('ascii')])

def openPort(serial_port):
    try:
        ser = serial.Serial(serial_port,115200)
    except:
        print('Error: Failed to open communications port, exiting')
        exit()
    else:
        ser.timeout=5
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        return ser

def serial_tx(sermsg):
    try:
        ser.write(bytearray([sermsg]))
    except:
        print("Sending command failed!")

def serial_rx_task():
    while True:
        try:
            serdata=ser.readline().decode('ascii').rstrip().split(" ")
        except:
            pass
        else:
            #if debug_level > 1:
            #    print(serdata)
            data=[]
            for i in range(2,len(serdata)):
                data.append(int(serdata[i],16))
            msg={'timestamp': time.time(), 'arbitration_id': int(serdata[1],16), 'dlc': len(serdata)-2, 'data': data}
            q.put(msg)

def serial_tx_heartbeat():
    global lastHeartbeat
    if args.noheartbeat:
        return
    if debug_level > 0:
        print("Sending heartbeat")
    try:
        ser.write(bytearray([commands['heartbeat']]))
    except:
        print("Sending heartbeat failed!")
    else:
        lastHeartbeat = int(time.time())

def main():
    global lastHeartbeat
    retain=False
    serial_tx_heartbeat()
    if(mqttOut==2):
        retain=True

    def getLine():
        if q.empty():  # Check if there is a message in queue
            return
    
        message = q.get()
        #if debug_level>0:
        #    print("{0:f} {1:d} ({2:d}) ".format(message['timestamp'], message['arbitration_id'], message['dlc']),end='')
        try:
            dev=incomingIds[message['arbitration_id']]
        except:
            print("Failed to find Device for CanID: {0:d}".format(message['arbitration_id']))
            return
        #if debug_level>0:
        #    print("Device: {0:s}: ".format(dev),end='',flush=True)
        for i in range(message['dlc']):
            field=statusPosition[dev][i]
            #if debug_level>0:
            #    print("{0:s} = {1:02x}, ".format(field,message['data'][i]),end='',flush=True)
            if field=='count':
                try:
                    lastStatus[dev][field]+=1
                except:
                    lastStatus[dev][field]=1
            else:
                lastStatus[dev][field]=message['data'][i]
            if mqttOut:
                mqttc.publish(dev+"/"+field,str(lastStatus[dev][field])+","+str(message['timestamp']),retain=retain)
        lastStatus[dev]["pktTime"]=message['timestamp']
        if debug_level>0:
            #print("")
            print("{0:s}: ".format(dev),end='')
            print(json.dumps(lastStatus[dev]))
        if mqttOut and field!='count':
            mqttc.publish(dev+"/status",json.dumps(lastStatus[dev]))

    def mainLoop():
        if mqttOut:
            mqttc.loop_start()
        while True:
            getLine()
            if int(time.time())-lastHeartbeat == 1:
                serial_tx_heartbeat()
            time.sleep(0.05)
        if mqttOut:
            mqttc.loop_stop()
    mainLoop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-b", "--broker", default = '127.0.0.1', help="MQTT Broker, default 127.0.0.1")
    parser.add_argument("-d", "--debug", default = 0, type=int, choices=[0, 1, 2], help="debug data")
    parser.add_argument("-m", "--mqtt", default = 0, type=int, choices=[0, 1, 2], help="Send to MQTT, 1=Publish, 2=Retain")
    parser.add_argument("-n", "--noheartbeat", default = 0, type=int, choices=[0, 1], help="Disable sending heartbeat (for testing)")
    parser.add_argument("-p", "--port", default = "/dev/ttyUSB0", help="Serial interface to use")
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
        ser = openPort(args.port)
    except OSError:
        print('Cannot find SocketCAN device.')
        exit()

    q = queue.Queue()    
    t = Thread(target = serial_rx_task)    # Start receive thread
    t.start()


    main()
