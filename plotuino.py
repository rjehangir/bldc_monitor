#!/usr/bin/python 

from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
import serial
import crcmod
import struct
import time

class SerialConnection:
	def openConnection(self,port,baudrate):
		self.ser = serial.Serial(port,baudrate,timeout=10)
	
	# This function is temporary for testing purposes.
	def readMessage(self):
		return np.random.normal(size=4)
		
	#def readMessage(self):
		#preamble = ['\xFF','\xFA'];
		#input = [0x00,0x00]
		#input[1] = ser.read(1)
		#while ( True ):
			#if ( input == preamble ):
				#break
			#else:
				#input[0] = input[1]
				#input[1] = ser.read(1)

		#Get length
		#length = ser.read(1)

		#Get data
		#data = ser.read(ord(length))

		#Get checksum
		#input = ser.read(2)
		#checksum = struct.unpack('H',input)[0]

		#Verify checksum
		#crc16 = crcmod.mkCrcFun(0x11021,0xFFFF,True)
		#calcChecksum = crc16(data)
		#calcChecksum = (~calcChecksum) % 2**16  # convert to uint16_t

		#if ( checksum != calcChecksum ):
			#print "Failed checksum."
			#continue
		      
		#Break data into useful parts
		#messageType = struct.unpack('H',data[0:2])[0]
		#numValues = int(length/4)
		#values = []
		#for i in range(numValues):
			#values.append(struct.unpack('f',data[4*i+2:4*i+6])[0])

		#return values


#######################################
app = QtGui.QApplication([])

sercon = SerialConnection()

win = pg.GraphicsWindow(title="Plotuino")
win.resize(1000,600)
win.setWindowTitle("Plotuino Test")

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)

plots = []
curves = []
timeData = np.ndarray(0)
yData = [np.ndarray(0),np.ndarray(0),np.ndarray(0),np.ndarray(0)]

plots.append(win.addPlot(title="Voltage"))
plots.append(win.addPlot(title="Current"))
win.nextRow()
plots.append(win.addPlot(title="Power"))
plots.append(win.addPlot(title="RPM"))

plots[0].setLabel('left',"Volts",units='V')
plots[1].setLabel('left',"Current",units='A')
plots[2].setLabel('left',"Power",units='W')
plots[3].setLabel('left',"RPM",units='RPM')

colors = ['r','g','b','y']

for i in range(len(plots)):
	curves.append(plots[i].plot(pen=colors[i]))
	
	plots[i].enableAutoRange('y', True)
	plots[i].enableAutoRange('x', False)
	
	plots[i].showGrid(x=True,y=True)
	plots[i].setLabel('bottom',"Time",units='s')
	
plots[1].setXLink(plots[0])
plots[2].setXLink(plots[0])
plots[3].setXLink(plots[0])

startTime = time.time()

def update():
	global timeData,yData
	values = sercon.readMessage()
	for i in range(len(yData)):
		yData[i] = np.append(yData[i],values[i])
	timeData = np.append(timeData,time.time()-startTime)
	if ( len(timeData) > 1000 ):
		timeData = np.delete(timeData,0,0)
		for i in range(len(yData)):
			yData[i] = np.delete(yData[i],0,0)
	
def redraw():
	global timeData,yData,curves
	for i in range(len(curves)):
		curves[i].setData(timeData,yData[i])
	for p in plots:
		vr = p.viewRange()
		xwidth = vr[0][1]-vr[0][0]
		p.setXRange(timeData[len(timeData)-1]-xwidth,timeData[len(timeData)-1],padding=0,update=False)
		p.enableAutoRange('y',True)
		p.setAutoPan(x=True)
	
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(100)

timer2 = QtCore.QTimer()	 
timer2.timeout.connect(redraw)
timer2.start(100)

if __name__ == '__main__':
	import sys
	if (sys.flags.interactive != 1) or not hasattr(QtCore,'PYQT_VERSION'):
		QtGui.QApplication.instance().exec_()