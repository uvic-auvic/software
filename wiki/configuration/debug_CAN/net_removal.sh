#!/bin/bash

# This script removes a network interface

echo What is the bus called?
read BUSNAME
echo removing $BUSNAME ...
sudo ip link del $BUSNAME
echo done.

