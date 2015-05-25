#!/usr/bin/env python
# Copyright (c) 2015 Universitaet Bremen - Institute for Artificial Intelligence (Prof. Beetz)
#
# Author: Minerva Gabriela Vargas Gleason mvargasg@uni-bremen.de
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

# DESCRIPTION: Communicate to the TDK Genesys power supply to obtain the output voltaje, current, etc and publish these in a ROS topic
# Init ROS
import rospy
from gen_pwr_source.msg import genesys_status
#Init Pyserial
import time
import serial

# configure the serial connections 
# Connect to....Direct to Com1 or Com 2
# Bits per second .......9600
# Data bits ................8
# Parity ................None
# Stop bits.................1
# Flow control...........None

ser = serial.Serial(
	port='/dev/ttyUSB0',
	baudrate=19200,
	timeout=1,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS
	)

# ser.open()
ser.isOpen()

def start_communication():
	ser.write('ADR 01\r')
	test=ser.read(10)

ser.write('ADR 01\r')
test=ser.read(10)

while test != 'OK\r':
	start_communication()

#while 1:
#	ser.write('ARD 06\r')
#	test=ser.read(10)
#	if test == 'OK\r':
#		break

rospy.init_node('power_source', anonymous=True)

pub = rospy.Publisher('genesys_status', genesys_status, queue_size=5)

msg=genesys_status()

# SST Reads the complete power supply status.
# ReturnsASCII characters representing the following data, separated by commas:
# MV<actual (measured) voltage> PV<programmed (set) voltage>
# MC<actual (measured) current> PC<programmed (set) current>
# SR<status register, 2-digit hex> FR<fault register, 2-digit hex>
# Example response: MV(45.201),PV(45),MC(4.3257),PC(10),SR(30),FR(00)

def read_values():
	while not rospy.is_shutdown():
		ser.write ('STT?\r')
		values=ser.read(65)

		mv_in=values.find('MV(')
		mv_end=values.find('),PV')
		m_voltage=float(values[(mv_in+3):mv_end]) # Measured Voltage

		pv_in=values.find('PV(')
		pv_end=values.find('),MC')
		p_voltage=float(values[(pv_in+3):pv_end]) # Programmed Voltage

		mc_in=values.find('MC(')
		mc_end=values.find('),PC')
		m_current=float(values[(mc_in+3):mc_end]) # Measured Current

		pc_in=values.find('PC(')
		pc_end=values.find('),SR')
		p_current=float(values[(pc_in+3):pc_end]) # Measured Current

		sr_in=values.find('SR(')
		sr_end=values.find('),FR')
		status_reg=int(values[(sr_in+3):sr_end],16) # Status Register

		fr_in=values.find('FR(')
		fault_reg=int(values[(mc_in+3):(mc_in+5)],16) # Fault Register

		# Messages
		msg.meas_voltage=m_voltage
		msg.prog_voltage=p_voltage
		msg.meas_current=m_current
		msg.prog_current=p_current
		# Status register
		msg.sr_constant_volt=bool(status_reg & 0x01)
		msg.sr_constant_curr=bool(status_reg & 0x02)
		msg.sr_no_fault=bool(status_reg & 0x04)
		msg.sr_fault=bool(status_reg & 0x08)
		msg.sr_auto_start=bool(status_reg & 0x10)
		msg.sr_fold_enabled=bool(status_reg & 0x20)
		msg.sr_local_mode=bool(status_reg & 0x80)
		# Fault Register
		msg.fr_ac_fail=bool(status_reg & 0x02)
		msg.fr_ovet_temp=bool(status_reg & 0x04)
		msg.fr_foldback=bool(status_reg & 0x08)
		msg.fr_over_volt_prot=bool(status_reg & 0x10)
		msg.fr_shutt_off=bool(status_reg & 0x20)
		msg.fr_output_off=bool(status_reg & 0x40)

		rospy.loginfo(msg)
		pub.publish(msg)

try:
	read_values()
except rospy.ROSInterruptException:
	pass


ser.close()

