#!/usr/bin/python 

from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
import serial
import crcmod
import struct
import time

import plotly
import json

class SerialConnection:
	def openConnection(self,port,baudrate):
		self.ser = serial.Serial(port,baudrate,timeout=0.25)
		time.sleep(1.0)
		if self.ser.isOpen():
			print "Serial port opened."
	
	## This function is temporary for testing purposes.
	def readMessageFake(self):
		return np.random.normal(size=6)
		
	def readMessage(self):
		preamble = ['\xFF','\xFA'];
		input = [0x00,0x00]
		input[1] = self.ser.read(1)
		while ( True ):
			if ( input == preamble ):
				break
			else:
				input[0] = input[1]
				input[1] = self.ser.read(1)

		#Get length
		length = ord(self.ser.read(1))
		
		while ( self.ser.inWaiting() < length + 2 ):
			pass

		#Get data
		data = self.ser.read(length)

		#Get checksum
		input = self.ser.read(2)
		checksum = struct.unpack('H',input)[0]

		#Verify checksum
		crc16 = crcmod.mkCrcFun(0x11021,0xFFFF,True)
		calcChecksum = crc16(data)
		calcChecksum = (~calcChecksum) % 2**16  # convert to uint16_t

		if ( checksum != calcChecksum ):
			print "Failed checksum."
			return
		      
		#Break data into useful parts
		messageType = struct.unpack('H',data[0:2])[0]
		numValues = int(length/4)
		values = []
		for i in range(numValues):
			values.append(struct.unpack('f',data[4*i+1:4*i+5])[0])

		return values


#######################################

sercon = SerialConnection()

connected = False
try:
	sercon.openConnection('/dev/ttyACM0',115200)
	connected = True
except:
	connected = False
	print "Error connecting to serial port. Defaulting to random data."

with open('./config.json') as config_file:
	plotly_user_config = json.load(config_file)

username = plotly_user_config['plotly_username']
api_key = plotly_user_config['plotly_api_key']
stream_tokens = plotly_user_config['plotly_streaming_tokens']
stream_server = 'http://stream.plot.ly'

p = plotly.plotly(username, api_key)

data = [{'x':[],'y':[],'type': 'scatter', 'stream': {'token': stream_tokens[0], 'maxpoints': 1000}}]

plotLayout = {'xaxis':{'range':[-1,1]}, 'yaxis': {'range': [-1,1]}}

r = p.plot(data, plotLayout, filename='Stream Example', fileopt='overwrite')

s = plotly.stream(stream_tokens)

s.write({'x':0.5,'y':0.5})

def update(pstream):
	if connected:
		values = sercon.readMessage()
	else:
		values = sercon.readMessageFake()
	if values is not None:
		ts = time.time()
		data = {'x':1,'y':values[0]}
		print data
		pstream.write(data)
			
if __name__ == '__main__':
	while True:
		update(s);
		time.sleep(0.1)
