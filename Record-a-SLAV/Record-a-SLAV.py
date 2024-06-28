#!/usr/bin/python3
import paho.mqtt.client as mqtt
from datetime import datetime
import gzip

""" USER CONFIG """

# Set up topics as defined in the MIROSLAV Arduino config
# of all active MIROSLAVs.
topics = [
    #"mph/env/rack_M", ## uncomment this to record environmental data, if the MIROSLAV is transmitting it
    #"mph/status/rack_R", ## uncomment this to record MIROSLAV status messages
    "mph/pir/rack_R",
    #"mph/status/rack_M", ## uncomment this to record MIROSLAV status messages
    "mph/pir/rack_M"
    ]

# Set the IP address of the computer serving as the MQTT broker
broker_ip = "192.168.1.166"

# Set the port, default is 1883
broker_port = 1883

# Print received messages to terminal in addition to logging them to a file.
# Useful as monitoring, can be problematic if you have many devices sending messages.
print_to_terminal = True

""" END USER CONFIG """

topic_files = {}

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    timenow = datetime.now().isoformat().replace(":","-").replace(".","-")
    for i, t in enumerate(topics):
        client.subscribe(t, qos=1)
        filename = t.replace("/", "-")+"."+timenow+".gz"
        topic_files[t] = gzip.open(filename, "wt")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    if print_to_terminal:
        print(msg.topic+" "+str(msg.payload))
    line = '"'+datetime.now().isoformat()+'";"'+msg.payload.decode("utf-8")+'"\n'
    topic_files[msg.topic].write(line)

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set(username="miroslav", password="hesoyam")
client.connect(broker_ip, broker_port, 60)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
try:
    client.loop_forever()
except KeyboardInterrupt:
    pass

print("Client stopped. Closing files...")
for f in topic_files:
    topic_files[f].close()

