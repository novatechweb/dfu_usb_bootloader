#!/usr/bin/python2
import sys, os

serial = sys.argv[1]

if len(serial) > 15:
	raise Exception("serial number too long")

print "Setting serial number to %s" % (serial)

data = ""
with open("stm32f1_dfu", "rb") as f:
	while True:
		byte = f.read(1)
		if byte == "":
			break
		data += byte

data = data.replace("SERIAL NUM HERE", serial.ljust(15,"\0"))

with open("stm32f1_dfu.serial", "wb") as f:
	f.write(data)


