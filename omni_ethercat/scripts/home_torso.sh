#!/bin/sh

#reset errors
ethercat download -p 4 0x6040 0 0x80 --type uint16

#get the drive almost up
ethercat download -p 4 0x6040 0 0x06 --type uint16
ethercat download -p 4 0x6040 0 0x07 --type uint16

#Set search speed
ethercat download -p 4 0x6099 1 200000 --type uint32
#set slow speed
ethercat download -p 4 0x6099 2 20000 --type uint32
#set deaccel
ethercat download -p 4 0x609A 0 10000000 --type uint32

#set homing mode to 2
ethercat download -p 4 0x6098 0 2 --type int8

#Home offset to zero
ethercat download -p 4 0x607c 0 0 --type int32

#Switch to mode of operation 6 (homing)
ethercat download -p 4 0x6060 0 6 --type int8

#Bring the drive up
ethercat download -p 4 0x6040 0 0x0f --type uint16


#Start homing:
ethercat download -p 4 0x6040 0 0x1f --type uint16

#When finished, switch to the other mode
#ethercat download -p 4 0x6060 0 1

