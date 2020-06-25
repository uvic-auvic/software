#!/bin/bash

# This script displays the data on the specified BUS

echo what is the bus name?
read BUSNAME


echo starting candump on bus $BUSNAME
candump $BUSNAME
