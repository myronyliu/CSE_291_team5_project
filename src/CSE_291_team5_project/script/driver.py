#!/usr/bin/env python
import rospy
import pici
import serial
import constant as C
from std_msgs.msg import Int8,Bool,Float32
import sys



VELOCITYCHANGE = 200
ANGULARCHANGE = 200


import struct
import  glob # for listing serial ports

def sendCommandRaw(sel,command):
    sel.write(command)


def sendCommandASCII(sel,command):
        cmd = ""
        for v in command.split():
            cmd += chr(int(v))
        sendCommandRaw(sel,cmd)


def cmd_full(sel):
    sendCommandASCII(sel,'132')


def cmd_passive(sel):    
    sendCommandASCII(sel, '128')
    
    

def mprint(x):
    print "[Error in Driver.py]" + x


def start_driver(sPort):
    sBaud = 115200              # Important!!!!
    try:        
        ser = serial.Serial(sPort,sBaud, timeout=1.0)
        cmd_full(ser);
        print("Connected to " + sPort + " successfully.")
    except serial.serialutil.SerialException as e:
        mprint("Not be able to to device:"+sPort)
        mprint("\nGet Error:"+str(e))
        return

    
    

        

    def topic_key_callback(msg):
        d = msg.data        
        if d == C.MOVE_DOWN:
            pici.drive_straight(ser,-VELOCITYCHANGE);
        if d == C.MOVE_UP:
            pici.drive_straight(ser,VELOCITYCHANGE);
        if d == C.MOVE_LEFT:
            pici.turn(ser,ANGULARCHANGE,cw=False)
        if d == C.MOVE_RIGHT:
            pici.turn(ser,ANGULARCHANGE,cw=True)
        if d == C.FULL:
            cmd_full(ser);
            print "full mode"
        if d == C.SONG:
            pici.playSongStart(ser)
            pici.playSongStop(ser)            
        if d == C.PASSIVE:
            cmd_passive(ser)
            print "passive mode"
        if d == C.STOP:
            pici.stop(ser);
            print "now stop"
        
        

    
    rospy.init_node('driver')
    rospy.Subscriber(C.DRIVER_ADDR, Int8, topic_key_callback);
    rospy.spin()
    

     
if __name__ == '__main__':
    # mprint(str(sys.argv))
    try:        
        if len(sys.argv) < 4:
            start_driver('/dev/ttyUSB0');
        else:
            start_driver(sys.argv[1]);
    except Exception as e:        
        print e
