#!/bin/bash

# This script displays the statiscs for a specific bus

echo what is the bus name?
read BUSNAME

ip -details -statistics link show $BUSNAME

echo done.
