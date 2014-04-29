#!/usr/bin/python 

import numpy as np
import serial
import crcmod
import struct
import time
import datetime
import plotly
import json
import sys

class SerialConnection:
	def openConnection(self,port,baudrate):
		self.ser = serial.Serial(port,baudrate,timeout=0.25,writeTimeout=0)
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
			#print "Failed checksum."
			return
		      
		#Break data into useful parts
		messageType = struct.unpack('H',data[0:2])[0]
		numValues = int(length/4)
		values = []
		for i in range(numValues):
			values.append(struct.unpack('f',data[4*i+1:4*i+5])[0])

		return values

class PlotlyPlotter:
	def initPlotly(self):
		with open('./config.json') as config_file:
			plotly_user_config = json.load(config_file)

		username = plotly_user_config['plotly_username']
		api_key = plotly_user_config['plotly_api_key']
		stream_tokens = plotly_user_config['plotly_streaming_tokens']
		stream_server = 'http://stream.plot.ly'

		self.p = plotly.plotly(username, api_key)

		numPoints = 1000

		trace0 = {'x':[],'y':[],'type': 'scatter', 'stream': {'token': stream_tokens[0], 'maxpoints': numPoints}}
		trace1 = {'x':[],'y':[],'yaxis':'y2','type': 'scatter', 'stream': {'token': stream_tokens[1], 'maxpoints': numPoints}}
		trace2 = {'x':[],'y':[],'yaxis':'y3','type': 'scatter', 'stream': {'token': stream_tokens[2], 'maxpoints': numPoints}}
		trace3 = {'x':[],'y':[],'yaxis':'y4','type': 'scatter', 'stream': {'token': stream_tokens[3], 'maxpoints': numPoints}}

		xAxisStyle = {'title':'Time'}

		domainHeight = 0.22
		domainGap = (1.0-4*domainHeight)/3.0

		layout1 = { 'title':'Thruster-100 Live Test Data',
			    'showlegend': False,
			    'xaxis':xAxisStyle,
			    'yaxis':{'domain':[0,domainHeight],'title':'Voltage (V)'},
			    'yaxis2':{'domain':[domainHeight+domainGap,2*domainHeight+domainGap],'title':'Power (W)'},
			    'yaxis3':{'domain':[2*domainHeight+2*domainGap,3*domainHeight+2*domainGap],'title':'RPM'},
			    'yaxis4':{'domain':[3*domainHeight+3*domainGap,4*domainHeight+3*domainGap],'title':'Thrust (lb)'}}

		r = self.p.iplot([trace0, trace1, trace2, trace3],layout=layout1,filename='Thruster-Stream',fileopt='overwrite')
		print r

		self.s0 = plotly.stream(stream_tokens[0])
		self.s1 = plotly.stream(stream_tokens[1])
		self.s2 = plotly.stream(stream_tokens[2])
		self.s3 = plotly.stream(stream_tokens[3])

		#timeStamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
		#self.s0.write({'x':timeStamp,'y':0.5})
		#self.s1.write({'x':timeStamp,'y':0.5})
		#self.s2.write({'x':timeStamp,'y':0.5})
		#self.s3.write({'x':timeStamp,'y':0.5})
	
	def streamToPlotly(self,data):
		timeStamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
		trace0 = {'x': timeStamp, 'y': data[0]}
		trace1 = {'x': timeStamp, 'y': data[1]}
		trace2 = {'x': timeStamp, 'y': data[2]}
		trace3 = {'x': timeStamp, 'y': data[3]}
		self.s0.write(trace0)
		self.s1.write(trace1)
		self.s2.write(trace2)
		self.s3.write(trace3)

#######################################

sercon = SerialConnection()

connected = False
try:
	sercon.openConnection('/dev/ttyACM0',115200)
	connected = True
except:
	connected = False
	print "Error connecting to serial port. Defaulting to random data."

plotter = PlotlyPlotter()

plotter.initPlotly()

import curses
stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
stdscr.nodelay(1)

stdscr.addstr(0,0,"Plotlyuino Thrust Test Stand")
stdscr.addstr(1,0,"============================")
stdscr.addstr(3,0,"Adjust motor:\tUp/down, pgup/pgdown, scroll wheel")
stdscr.addstr(4,0,"Stop motor: \tSPACE")
stdscr.addstr(5,0,"Quit: \t\tq")
if connected:
	stdscr.addstr(15,0,"Live serial data.")
else:
	stdscr.addstr(15,0,"Simulated data.")
stdscr.refresh()

command = 0
streamCount = 0

def getMotorFromTerminal():
	global command
	
	key = stdscr.getch()
	
	if key == curses.KEY_UP:
		  command += 1
		  if command > 255:
			  command = 255
	elif key == curses.KEY_DOWN:
		  command -= 1
		  if command < 0:
			  command = 0
	elif key == curses.KEY_PPAGE:
		  command += 10
		  if command > 255:
			  command = 255
	elif key == curses.KEY_NPAGE:
		  command -= 10
		  if command < 0:
			  command = 0
	elif key == ord(' '):
		  command = 0
	elif key == ord('q'):
		  command = 0
		  if connected:
			sercon.ser.write(chr(command))
		  curses.endwin()
		  exit()
		  
	stdscr.addstr(8,5,"Motor command: "+str(command)+"     ")

def update():
	if connected:
		values = sercon.readMessage()
	else:
		values = sercon.readMessageFake()
	if values is not None:
		#print values
		global streamCount
		streamCount += 1
		plotter.streamToPlotly(values)
		stdscr.addstr(10,5,"Thrust:\t%10.2f lb\t%10.0f g"%(values[3],values[3]*453.6))
		stdscr.addstr(11,5,"RPM:\t%10.2f rev/min"%(values[2]))
		stdscr.addstr(12,5,"Power:\t%10.2f W"%(values[1]))
		stdscr.addstr(13,5,"Voltage:\t%10.2f V"%(values[0]))
		stdscr.addstr(16,0,"Plotly Stream Data Points Sent: %10.0f"%(streamCount))

			
if __name__ == '__main__':
	lastCommandUpdate = time.time()
	lastCommand = 0
	
	lastSerialRead = time.time()
	while True:
		if time.time() - lastSerialRead > 0.18:
			update()
			lastSerialRead = time.time()
			
		getMotorFromTerminal()
		
		if time.time() - lastCommandUpdate > 0.1 and command is not lastCommand and connected:
			sercon.ser.write(chr(command))
			lastCommand = command
			lastCommandUpdate = time.time()

