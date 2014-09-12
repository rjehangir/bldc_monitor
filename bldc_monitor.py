#!/usr/bin/python 

import numpy as np
import serial
import crcmod
import struct
import time
import datetime
import plotly.plotly as py
from optparse import OptionParser
import json
import signal
import sys
import csv

class SerialConnection:
	def openConnection(self,port,baudrate):
		self.ser = serial.Serial(port,baudrate,timeout=0.05,writeTimeout=0)
		time.sleep(1.0)
		if self.ser.isOpen():
			print "Serial port opened."
	
	## This function is temporary for testing purposes.
	def readMessageFake(self):
		formatString = 'HHfff'
		return np.random.normal(size=struct.calcsize(formatString))
		
	def read(self,length):
		preamble = ['\xFF','\xFA'];
		input = [0x00,0x00]
		input[1] = self.ser.read(1)
		while ( True ):
			if ( input == preamble ):
				break
			else:
				input[0] = input[1]
				input[1] = self.ser.read(1)
		
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
		      
		return data

class PlotlyPlotter:
	def initPlotly(self):
		with open('./config.json') as config_file:
			plotly_user_config = json.load(config_file)

		username = plotly_user_config['plotly_username']
		api_key = plotly_user_config['plotly_api_key']
		stream_tokens = plotly_user_config['plotly_streaming_tokens']
		stream_server = 'http://stream.plot.ly'

		py.sign_in(username, api_key)

		numPoints = 450

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

		r = py.iplot({'data':[trace0, trace1, trace2, trace3],'layout':layout1},filename='Thruster-Stream',fileopt='overwrite')
		print r

		self.s0 = py.Stream(stream_tokens[0])
		self.s1 = py.Stream(stream_tokens[1])
		self.s2 = py.Stream(stream_tokens[2])
		self.s3 = py.Stream(stream_tokens[3])
		
		self.s0.open()
		self.s1.open()
		self.s2.open()
		self.s3.open()
		
	def closePlotly(self):
		self.s0.close()
		self.s1.close()
		self.s2.close()
		self.s3.close()
	
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
		
class CumulativeTimeMeter:
		def init(self):
				self.startTime = time.time()
				self.fileName = 'hourmeter.log'
				with open(self.fileName,'r') as f:
						try:
								oldRecord = json.load(f)
						except ValueError, e:
								oldRecord = {'cumulativeTime':0,'cumulativeRPSxTime':0,'cumulativeThrustxTime':0}
								with open(self.fileName,'w') as f2:
										json.dump(oldRecord,f2)
				self.oldTime = oldRecord['cumulativeTime']
				self.oldRPSTime = oldRecord['cumulativeRPSxTime']
				self.oldThrustTime = oldRecord['cumulativeThrustxTime']
				self.time = 0
				self.rpsTime = 0
				self.thrustTime = 0
				self.lastTime = time.time()
				
		def getCumulativeTime(self):
				return self.time+self.oldTime
				
		def getCumulativeRPSxTime(self):
				return self.rpsTime + self.oldRPSTime
				
		def getCumulativeThrustxTime(self):
				return self.thrustTime + self.oldThrustTime		
				
		def meterTime(self,isMetering,rpm,thrust):
				delta = time.time() - self.lastTime
				self.lastTime = time.time()
				
				if isMetering:
						self.time += delta
						self.rpsTime += delta*rpm/60.0
						self.thrustTime += delta*thrust
				
		def recordCumulativeTime(self):
				record = {'cumulativeTime':self.getCumulativeTime(),'cumulativeRPSxTime':self.getCumulativeRPSxTime(),'cumulativeThrustxTime':self.getCumulativeThrustxTime()}
				with open(self.fileName,'w') as f:
						json.dump(record,f)
				

#######################################

meter = CumulativeTimeMeter()

meter.init()

sercon = SerialConnection()

parser = OptionParser()
parser.add_option("-o","--output",dest="filename",help="Output file name.",metavar="FILE")
parser.add_option("-p","--port",dest="portname")
(options,args) = parser.parse_args()

isFile = False
if options.filename:
	csvfile = open(options.filename,'w')
	csvwriter = csv.writer(csvfile,delimiter=',')
	isFile = True

connected = False
try:
	sercon.openConnection(options.portname,57600)
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

stdscr.addstr(0,0,"Thrust Test Stand")
stdscr.addstr(1,0,"============================")
stdscr.addstr(3,0,"Adjust motor:\tUp/down, pgup/pgdown, scroll wheel")
stdscr.addstr(4,0,"Stop motor: \tSPACE")
stdscr.addstr(5,0,"Quit: \t\tq")
if connected:
	stdscr.addstr(15,0,"Live serial data.")
else:
	stdscr.addstr(15,0,"Simulated data.")
stdscr.refresh()

command = 1500
streamCount = 0
values = []

def sigint_handler(*args):
	print "Closing serial port."
	sercon.ser.close()
	print "Quiting gracefully."
	time.sleep(0.25)
	sys.exit()

def getMotorFromTerminal():
	global command
	
 	key = stdscr.getch()
	
	if key == curses.KEY_UP:
		  command += 5
		  if command > 1900:
			  command = 1900
	elif key == curses.KEY_DOWN:
		  command -= 5
		  if command < 1100:
			  command = 1100
	elif key == curses.KEY_PPAGE:
		  command += 50
		  if command > 1900:
			  command = 1900
	elif key == curses.KEY_NPAGE:
		  command -= 50
		  if command < 1100:
			  command = 1100
	elif key == ord(' '):
		  command = 1500
	elif key == ord('q'):
		  command = 1500
		  if connected:
			sercon.ser.write(chr(command))
		  plotter.closePlotly()
		  curses.endwin()
		  sercon.ser.close()
		  exit()
		  
	stdscr.addstr(8,5,"Motor command: "+str(command)+"     ")

def readSerial():
	global values
	if connected:
		values = sercon.readMessage()
	else:
		values = sercon.readMessageFake()

def updateCurses():
	stdscr.addstr(10,5,"Thrust:\t%10.2f lb\t%10.0f g"%(values[5],values[5]*453.6))
        stdscr.addstr(11,5,"RPM:\t%10.0f rev/min"%(values[0]))
        stdscr.addstr(12,5,"Power:\t%10.0f W"%(values[2]*values[4]))
        stdscr.addstr(13,5,"Voltage:\t%10.2f V"%(values[4]))
        stdscr.addstr(14,5,"PWM:    \t%g\t%g us"%(values[6],values[7]))
        stdscr.addstr(16,0,"Plotly Stream Data Points Sent: %10.0f"%(streamCount))

def updatePlotly():
	global values
	if values is not None:
		global streamCount
		streamCount += 1
		plotter.streamToPlotly(values)
		
def updateHourMeter():
	global values
	isMetering = False
	if values[0] > 60:
		isMetering = True
	meter.meterTime(isMetering,values[0],values[5])
	stdscr.addstr(20,0,"Cumulative Running Time:\t%s"%(str(datetime.timedelta(seconds=meter.getCumulativeTime()))))
	stdscr.addstr(21,0,"Cumulative Revolutions:\t\t%10.0f"%(meter.getCumulativeRPSxTime()))
	stdscr.addstr(22,0,"Average Thrust:\t\t\t%10.2f"%(meter.getCumulativeThrustxTime()/(meter.getCumulativeTime()+0.001)))
	meter.recordCumulativeTime()
			
if __name__ == '__main__':
	signal.signal(signal.SIGINT, sigint_handler)

	lastCommandUpdate = time.time()
	lastCommand = 0
	lastSerialRead = time.time()
	lastPlotlyUpdate = time.time()
	lastMeterRecord = time.time()
	errorCount = 0

	formatString = 'HHffffHH'
	bldcdata = []

	if isFile:
		csvwriter.writerow(["rpma","rpmb","currentA","currentB","voltage","thrust","pwmA","pwmB"])

	sendCount = 0

	while True:
		if time.time() - lastSerialRead > 0.05:
			lastSerialRead = time.time()
			length = struct.calcsize(formatString)
			bldcdata = sercon.read(length)
			if bldcdata is not None:
				values = struct.unpack(formatString,bldcdata)
				updateCurses()
				if isFile:
					csvwriter.writerow(values)
			
		if False and (time.time() - lastPlotlyUpdate > 0.50):
			try:
				updatePlotly()
			except IOError:
				errorCount += 1
				time.sleep(5)
			lastPlotlyUpdate = time.time()
			
		if False and time.time() - lastMeterRecord > 0.20:
			lastMeterRecord = time.time()
			updateHourMeter()
			

		getMotorFromTerminal()
		getMotorFromTerminal()
		
		if (( time.time() - lastCommandUpdate > 0.1 and command is not lastCommand ) or (time.time()-lastCommandUpdate > 0.5)) and connected:
			stdscr.addstr(30,0,"Sent: \t\t\t%g"%(sendCount))
			sendCount += 1
			sercon.ser.write('\xFF')
			sercon.ser.write('\xFA')
			txData = struct.pack('HH',command,command)
			crc16 = crcmod.mkCrcFun(0x11021,0xFFFF,True)
			calcChecksum = crc16(txData)
			calcChecksum = (~calcChecksum) % 2**16  # convert to uint16_t
			for i in range(len(txData)):
				sercon.ser.write(txData[i])
			sercon.ser.write(struct.pack('H',calcChecksum)[0])
			sercon.ser.write(struct.pack('H',calcChecksum)[1])

			lastCommand = command
			lastCommandUpdate = time.time()

