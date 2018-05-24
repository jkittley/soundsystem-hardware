# Sound System - Lab Study Code
This code in this directory was used to run the lab study. All nodes are on Network 50 to avoid confusion with other field nodes in operation.

## simple_tx.ino
This script sends a message to a specified node as often as possible. It does not request an acknowledgement.

## rssi_logger.ino
This script listens for messages sent by `simple_tx.ino`and outputs the RSSI value to the serial port. This script can be used to measure the signal strength a node will see in a room. The assumption is that the simple_tx.ino sketch is running on a device identical to the logger i.e. the RSSI will be as simmular as possible to a ping pong setup, where the pong returns the RSSI of the gateway.

## node.ino
This sketch listens for messages from a TX node (i.e. a device running `simple_tx.ino`) and stores the RSSI, listens to a microphone and records the sound level in dB, and converts both of the values into values between 0 and 5 before forwards them on to the Relay Node via RFM69.

## relay.ino
Listens for messages send by the node and forwards the data on via BLE to the tablet running the SoundSystem app.
