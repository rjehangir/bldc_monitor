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
		return float(np.random.normal(size=1))
		
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

		## Get length
		#length = ser.read(1)

		## Get data
		#data = ser.read(ord(length))

		## Get checksum
		#input = ser.read(2)
		#checksum = struct.unpack('H',input)[0]

		## Verify checksum
		#crc16 = crcmod.mkCrcFun(0x11021,0xFFFF,True)
		#calcChecksum = crc16(data)
		#calcChecksum = (~calcChecksum) % 2**16  # convert to uint16_t

		#if ( checksum != calcChecksum ):
			#print "Failed checksum."
			#continue
		      
		## Break data into useful parts
		#messageType = struct.unpack('H',data[0:2])[0]
		#value = struct.unpack('f',data[2:6])[0]

		#return value


#######################################
app = QtGui.QApplication([])

sercon = SerialConnection()

win = pg.GraphicsWindow(title="Plotuino")
win.resize(1000,600)
win.setWindowTitle("Plotuino Test")

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)

p1 = win.addPlot(title="Test Plot")
curve1 = p1.plot(pen='b')
ydata = np.random.normal(size=0)
xdata = ydata*0
curve1.setData(xdata,ydata)

startTime = time.time()

def update():
	global xdata, ydata, curve1
	value = sercon.readMessage()
	ydata = np.append(ydata,value)
	xdata = np.append(xdata,time.time()-startTime)
	if ( len(ydata) > 1000 ):
		xdata = np.delete(xdata,0,0)
		ydata = np.delete(ydata,0,0)
	
def redraw():
	global xdata, ydata, curve1
	curve1.setData(xdata,ydata)
	
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