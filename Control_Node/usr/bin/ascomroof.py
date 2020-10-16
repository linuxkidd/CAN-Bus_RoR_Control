#!/usr/bin/env python3

import array,json,os,re,signal,sys,time
import paho.mqtt.client as mqtt
import ruamel.yaml as yaml

mypid=os.getpid()

debug_level=0
s={}

commands={'open':1,'close':1,'stop':1}
devs={'both':1,'roof':1,'flap':1}

states={ 'open':0, 'close':1, 'closed':1, 'opening':2, 'closing':3, 'err':4, 'heartbeat':254, 'stop':255 }

with open('/etc/ascomroof.yml','r') as stepfile:
  try:
    steps=yaml.round_trip_load(stepfile)
  except yaml.YAMLError as err:
    print(err)
    exit(1)

def signal_handler(signal, frame):
    print('')
    print('You pressed Ctrl+C!  Exiting...')
    print('')
    exit(0)

signal.signal(signal.SIGINT, signal_handler)

def on_mqtt_connect(client, userdata, flags, rc):
    if debug_level:
        print("MQTT Connected with code "+str(rc))
    if rc==0:
        try:
            client.subscribe([("flap/status",0),("roof/status",0),("flap/command",0),("roof/command",0),("ascomroof/pid",0)])
            client.loop_start()
            client.connected=True
            client.publish("ascomroof/pid",mypid)
        except Exception as e:
            print("Error: "+str(e))

def on_mqtt_subscribe(client, userdata, mid, granted_qos):
    if debug_level:
        print("MQTT Sub: "+str(mid))

def on_mqtt_message(client, userdata, msg):
    topic_parts=msg.topic.split("/")
    dev=topic_parts[0]
    if debug_level:
        print("Received "+msg.topic+" value "+msg.payload.decode('ascii'))
    if dev=="ascomroof":
        if int(msg.payload.decode('ascii'))!=mypid:
            print("Another instance of this script started under pid: "+msg.payload.decode('ascii')+" != "+str(mypid)+". Exiting for safety.")
            os.kill(os.getpid(), signal.SIGTERM)

    if topic_parts[1]=="command":
        if msg.payload.decode('ascii')=="stop" and cmd!="stop":
            print("Received STOP command for "+dev+", exiting for safety.")
            client.loop_stop()
            client.disconnect()
            os.kill(os.getpid(), signal.SIGTERM)
        return
    try:
        s[dev]=json.loads(msg.payload.decode('ascii'))
    except Exception as e:
        print("Error loading json: "+str(e))

def sendCommand(dev,cmd):
    if debug_level:
        print("Sending "+cmd+" for "+dev)
    try:
        mqttc.publish(dev+"/command",cmd)
        return 1
    except:
        print("Failed to send "+cmd+" for "+dev)
        return 0

def waitForState(dev,field,state,timeout=180):
    max_mqtt_wait=5
    if debug_level:
        print("Waiting up to "+str(max_mqtt_wait)+" seconds for MQTT status for "+dev)
    while dev not in s.keys():
        time.sleep(1)
        max_mqtt_wait-=1
        if max_mqtt_wait==0:
            print("Error: Timed out waiting for MQTT state of "+dev+".")
            exit(1)
    if dev in s.keys():
        if time.time()-s[dev]['pktTime']>3:
            print("Error: MQTT state of "+dev+" is stale ( > 3 seconds old ).")
            exit(1)
        waitedsecs=0
        if field not in s[dev]:
            print("Error: "+field+" is not a property of "+dev)
            exit(1)
        while waitedsecs<=timeout:
            if s[dev][field]!=state:
                time.sleep(1)
                waitedsecs+=1
            else:
                return 1 # success
        return 0 # fail

def usage():
    print("Usage:")
    print("\t"+sys.argv[0]+" <command> <device>\n")
    print("\t\t<command> is one of: "+", ".join(commands.keys()))
    print("\t\t<device>  is one of: "+", ".join(devs.keys())+"\n\n")
    exit(1)

if len(sys.argv)<3:
    usage()

mqttc=mqtt.Client()
mqttc.connected=False
mqttc.on_connect = on_mqtt_connect
mqttc.on_subscribe = on_mqtt_subscribe
mqttc.on_message = on_mqtt_message

try:
    mqttc.connect("127.0.0.1", port=1883)
    mqttc.loop_start()
except:
    print("MQTT Broker Connection Failed.  Exiting")
    exit(1)

if sys.argv[1] in commands:
    cmd=sys.argv[1]
else:
    usage()

if sys.argv[2] in devs:
    dev=sys.argv[2]
else:
    usage()

max_mqtt_wait=5
if debug_level:
    print("Waiting for up to "+str(max_mqtt_wait)+" seconds for MQTT Connect: ",end="",flush=True)

while mqttc.connected==False and max_mqtt_wait:
    if debug_level:
        print(".",end="",flush=True)
    time.sleep(0.1)
    max_mqtt_wait-=0.1
print("")

if max_mqtt_wait==0:
    print("Timed out waiting for MQTT Connect.")
    exit(1)

for step in steps[dev][cmd]:
    pastcmd=cmd
    if cmd=='close':
        pastcmd='closed'
    if step['checkonly']:
        if waitForState(step['dev'],'state',states[cmd],step['maxwait'])==0:
            print("Error: {0:s} is not {1:s}.".format(step['dev'], pastcmd))
            exit(1)
    else:
        if sendCommand(step['dev'],cmd)==0:
            exit(1)
        if step['maxwait']>-1:
            if waitForState(step['dev'],'state',states[cmd],step['maxwait'])==0:
                print("Error: {0:s} did not {1:s} in {2:d} seconds.".format(step['dev'], cmd, step['maxwait']))
                exit(1)
mqttc.loop_stop()
mqttc.disconnect()
exit(0)
