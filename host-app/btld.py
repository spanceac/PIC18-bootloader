import serial
import sys

f = open(sys.argv[2], 'r')
ser = serial.Serial(sys.argv[1], 115200, timeout = None)

recv = '1' # initialise char

for line in f:
	print 'Sending %s' % line,
	ser.write(line,)
	print 'Waiting for XON'
	recv = ser.read()
	while recv[0].encode('hex') != "11":
		print 'We received 0x%s waiting for XON' % recv[0].encode('hex')
		recv = ser.read()
	print 'Received XON ... continuing'

print 'Finnished sending hex'
ser.close()
