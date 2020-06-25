#!/bin/bash

# This script generates random frames on the specified bus

echo what is the bus name?
read BUSNAME


echo starting cangen on bus $BUSNAME ...
cangen $BUSNAME
