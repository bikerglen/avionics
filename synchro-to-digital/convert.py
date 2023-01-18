# Connections:
#
# Ch1 is Vr1-Vr2 (red/wht - blk/wht) (from eq set 1)
# Ch2 is Vs1-Vs3 (ylw - blu)         (from eq set 2)
# Ch3 is Vs3-Vs2 (blu - blk)         (from eq set 2)
# Ch4 is Vs2-Vs1 (blk - ylw)         (from eq set 2)
#
# Ch1 needs to be inverted to get the correct angle output
# since Vr1-Vr2 is captured but Vr2-Vr1 is needed.
#
# computing the error as the following results in an angle of 0 at 0 feet on the altitude 
# alerter with the angle increasing as the altitude increases:
#     delta = sinin*math.cos(3*math.pi/2 - theta) - cosin*math.sin(3*math.pi/2 - theta)
# this is technically not the correct result but it looks prettier when demoing the system

# imports

import serial
import time
import sys
import struct
import math

#----------------------------------------
# constants

DATAQ_SER_PORT = '/dev/ttyACM0' # DI-2108 serial port
DISP_SER_PORT  = '/dev/ttyUSB0' # serial display serial port
USE_SER_DISP   = 1

#----------------------------------------
# function to write command to dataq then wait up until the timeout 
# for the command string to be echoed back to the hose

def WriteCommandWait (serial, cmd, timeout):
    serial.write(cmd)
    start = time.time ()
    s = b"";
    while not (cmd in s) and time.time () - start < timeout:
        if (serial.in_waiting > 0):
            s = s + serial.read (1)

#----------------------------------------
# open serial ports

serDataq = serial.Serial (port = DATAQ_SER_PORT, timeout=0.5)
if USE_SER_DISP:
    serDisp  = serial.Serial (port = '/dev/ttyUSB0', baudrate = 115200)

#----------------------------------------
# configure dataq to capture channels 0 to 3 at 40 ksps each

WriteCommandWait (serDataq, b"stop\r", 0.25)      # stop in case device was left scanning
WriteCommandWait (serDataq, b"reset\r", 0.25)     # reset in case of any errors
WriteCommandWait (serDataq, b"encode 0\r", 0.25)  # set up the device for binary mode
WriteCommandWait (serDataq, b"slist 0 0\r", 0.25) # scan list position 0 channel 0
WriteCommandWait (serDataq, b"slist 1 1\r", 0.25) # scan list position 1 channel 1
WriteCommandWait (serDataq, b"slist 2 2\r", 0.25) # scan list position 2 channel 2
WriteCommandWait (serDataq, b"slist 3 3\r", 0.25) # scan list position 3 channel 3

WriteCommandWait (serDataq, b"filter 0 0\r", 0.25) 
WriteCommandWait (serDataq, b"filter 1 0\r", 0.25) 
WriteCommandWait (serDataq, b"filter 2 0\r", 0.25) 
WriteCommandWait (serDataq, b"filter 3 0\r", 0.25) 

WriteCommandWait (serDataq, b"srate 6000\r", 0.25) 
WriteCommandWait (serDataq, b"dec 1\r", 0.25) 
WriteCommandWait (serDataq, b"deca 1\r", 0.25)
WriteCommandWait (serDataq, b"ps 0\r", 0.25)

count = 0
theta = 0

print ('done with config ... starting')

try:
    # start data acquisition
    serDataq.reset_input_buffer ()
    serDataq.write (b"start 0\r")

    # loop forever
    while True:

        # wait for 8 bytes to be available
        i = serDataq.in_waiting
        if i >= 8:

            # read bytes and convert to array of channels
            response = serDataq.read (8)
            response2 = bytearray (response)
            channels = struct.unpack ("<"+"h"*4, response2)
    
            # break out channels, invert Vref to get Vr2-Vr1
            # note that s2ms1 isn't actually used
            r2mr1 = -channels[0] / 32768.0 # Vr2mr1 = -Vr1mr2 = -(Vredwht - Vblkwht)
            s1ms3 =  channels[1] / 32768.0 # Vs1ms3 = Vylw    - Vblu
            s3ms2 =  channels[2] / 32768.0 # Vs3ms2 = Vblu    - Vblk
            s2ms1 =  channels[3] / 32768.0 # Vs2ms1 = Vblk    - Vylw
    
            # scott t transform the inputs
            # sinin = s1ms3
            sinin = s1ms3
            # cosin = 2/sqrt(3) * (s3ms2 + 0.5 * s1ms3)
            cosin = 1.1547 * (s3ms2 + 0.5 * s1ms3)
    
            # convert reference waveform into square wave by getting its sign
            if (r2mr1 < 0):
                refsqwv = -1
            elif (r2mr1 > 0):
                refsqwv = +1
            else:
                refsqwv = 0
    
            # compute error
            delta = sinin*math.cos(theta) - cosin*math.sin(theta)
    
            # demodulate AC error term
            demod = refsqwv * delta
    
            # apply gain term to demodulated error term and integrate
            # theta = theta + 1/64*demod
            theta = theta + 0.015625*demod
    
            # wrap from -pi to +pi
            theta = ((theta+math.pi) % (2*math.pi)) - math.pi
    
            # display output at 100 Hz
            count = count + 1
            if count >= 400:
                count = 0
                if USE_SER_DISP:
                    serDisp.write("{:8.2f}\n".format(math.degrees(theta)).encode())
                    serDisp.reset_input_buffer()
                print ('{:5.1f}'.format (float(round(10.0*math.degrees(theta)))/10))
    
except KeyboardInterrupt:
    print ("stopping")
    serDataq.write(b"stop\r") 
    time.sleep(0.5)
    serDataq.write(b"reset\r") 
    time.sleep(0.5)
