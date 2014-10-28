#!/usr/bin/python 

from optparse import OptionParser

parser = OptionParser()
parser.add_option("-i","--input",dest="jsonSettings",help="Input settings file, JSON format.",metavar="FILE")
parser.add_option("-c","--clear",action="store_true",dest="clearFlag",default=False,help="Clear data from server")
(options,args) = parser.parse_args()

import json
settings = json.load(open(options.jsonSettings))

if options.clearFlag:
	import requests
	r = requests.get('http://data.sparkfun.com/input/'+settings["dataPublicKey"]+'/clear?private_key='+settings["dataPrivateKey"])
	print r.url
	print r.content
	exit()

from pprint import pprint
pprint(settings)

import serial
port = serial.Serial(settings["port"],settings["baudrate"],timeout=0.5,writeTimeout=0)

import requests
getURL = 'http://data.sparkfun.com/input/'+settings["dataPublicKey"]

import time
lastCommand = time.time()

import sys

while True:
	try:
		data = port.read(256)
	except IOError:
		print "Unexpected error:", sys.exc_info()[0]

	if len(data) > 0:
		try: 
			print data
			r = requests.get(getURL+'?private_key='+settings["dataPrivateKey"]+'&'+data)
			print r.text
		except ValueError:
			print "Unexpected error:", sys.exc_info()[0]

	if time.time() - lastCommand > 2.0:
		lastCommand = time.time()
		try:	
			r = requests.get('http://data.sparkfun.com/output/'+settings['cmdPublicKey']+'.json')
			command = r.json()

			print "PulseWidths: "+command[0]["pulsewidtha"]+", "+command[0]["pulsewidthb"]

			commandOut = "a"+command[0]["pulsewidtha"]+"\nb"+command[0]["pulsewidthb"]+"\n"
			print "Sending:\n" + commandOut

			port.write(str(commandOut))
		except ValueError:
			print "Unexpected error:", sys.exc_info()[0]