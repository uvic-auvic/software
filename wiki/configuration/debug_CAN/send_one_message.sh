#!/bin/bash

# This script sends a single message to a CAN bus

echo To what bus would you lke to send the message?
read BUSNAME

echo What is your message?
read MESSAGE

cansend $BUSNAME $MESSAGE

echo done.
