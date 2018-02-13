import serial
import sys

def check_if_err(recv):
	if recv == "G":
		print "Error: first instruction is not a GOTO, exiting"
		sys.exit(1)
	elif recv == "X":
		print "Error: this hex overwrites the bootloader zone, exiting"
		sys.exit(1)
	elif recv == "E":
		print "Error: MCU reported bad checksum, exiting"
		sys.exit(1)
	else:
		return

def main():
	recv = '1' # initialise char
	f = open(sys.argv[2], 'r')
	ser = serial.Serial(sys.argv[1], 115200, timeout = None)

	for line in f:
		print 'Sending %s' % line,
		ser.write(line,)
		print 'Waiting for XON'
		recv = ser.read()
		while recv[0].encode('hex') != "11":
			print 'We received 0x%s waiting for XON' % recv[0].encode('hex')
			check_if_err(recv)
			recv = ser.read()
		print 'Received XON ... continuing'

	print 'Finnished sending hex'
	ser.close()
	sys.exit(0)

if __name__ == "__main__":
    main()
