'''
Python iRobot Create Simulator

Includes the class "Create" which is the main object for simulation.

Pronounced "peaches".

'''
#import ipdb
import sys, datetime, time, os, signal, getopt
from math import *
from pylab import *
import threading as thrd
#from threading import Timer
#from threading import Thread
import logging
logger = logging.getLogger(__name__)
import pici
reload (pici)

class Beacon(object):
    '''
    '''
    def __init__(self, x, y, rgb, name):
        self.x = x
        self.y = y
        self.rgb = rgb
        self.name = name


class Wall(object):
    '''
    Wall object consists of simply a start point (x1, y1) and and 
    endpoint (x2, y2)
    Walls can by physical walls or virtual walls - they both use the same object.
    '''
    def __init__(self,x1,y1,x2,y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

class cMap(object):
    '''
    Object to hold the iRobot Create's map.

    Maps can be made via a text file or made in Python

    Args:
        mapfname : string, textfile name for reading the map
    '''
    def __init__(self,mapfname=None, name=None):
        self.walls = []
        self.virtwalls = []
        self.beacons = []
        self.name = 'None'
        if name is not None:
            self.name = name
        elif (name is None) and (mapfname is not None):
            self.name = mapfname
        
            
            
            
            
        if mapfname is not None:
            try:
                self.loadMap(mapfname)
            except:
                logger.error("There was an error reading the map from <%s>"%
                             mapfname)
            
    def add_wall(self, x1,y1,x2,y2):
        ''' 
        Add a wall to the map
        
        Args:
            x1, y1 : floats, start point [m]
            x2, y2 : floats, end point [m]
        '''
        self.walls.append(Wall(x1,y1,x2,y2))

    def add_virtwall(self, x, y, theta):
        '''
        Add a virtual wall

        Args:
            x, y : floats, start point [m]
            thets : float, direction of wall [deg]

        Note - assumed lengt of wall is 2 m
        
        '''
        x1 = x
        y1 = y
        L = 2.0  # assumed length of wall
        x2 = x1+L*cos(theta*pi/180.0)
        y2 = y1+L*sin(theta*pi/180.0)
        self.virtwalls.append(Wall(x1,y1,x2,y2))

    def loadMap(self,mapfname):
        '''
        Load a map from a text file.
        Textfile follows the same format as the MATLAB SimulatorGUI
        
        Args:
            mapfname : string, file name of the file to read
        '''
        if not os.path.isfile(mapfname):
            logger.error("Could not load the map from <%s>, appears the file doesn't exist"%mapfname)
            return
            
        mapF = open(mapfname, 'r')
        for line in mapF:
            ll = line.split()
            if len(ll) > 0:
                tag = ll[0]
                if not (tag[0]=='%'):
                    if (tag.lower() == 'wall') and (len(ll)==5):
                            self.add_wall(float(ll[1]),float(ll[2]),
                                          float(ll[3]),float(ll[4]))
                    elif (tag.lower() == 'virtwall') and (len(ll)==4):
                        self.add_virtwall(float(ll[1]),float(ll[2]),
                                          float(ll[3]))
        mapF.close()
        
    


class Create(object):
    '''
    This is the object that we'll create to substitute for the 
    serial.Serial object.  This is what the interface will write to and
    read from.
    '''

    def __init__(self, cmap=None, timeout = 1.0):
        '''
        Constructor
        '''
        self.timeout = timeout
        self.OImode = None

        # Map
        self.cmap = cmap

        # State of the Create
        self.vel_left = 0   # left wheel tranlational velocity [mm/s]
        self.vel_right = 0   # left wheel tranlational velocity [mm/s]
        self.X = 0 # x-position of roomba [m]
        self.Y = 0 # y-position of roomba [m] 
        self.A = 0 # angle of roomba [deg, ccw, zero is coincident with x]
        # Sensors
        self.BumpRight = False
        self.BumpLeft = False
        self.BumpFront = False
        self.VirtWall = False
        # Message to write when getting a query
        self.Msg = ""
        # These two are since the last request for sensor data
        self.distance = 0  # distance traveled [mm], mean dist. left and right
        self.angle = 0 # angle turned [deg]
                
        self.L = 258.0   # distance between wheels [mm]

        # Timer
        self.DT = 0.1
        self.timer = None
        self.timer_is_running = False
        # Auto-start
        self.T0 = time.time()
        self.start()

    def run(self):
        ''' 
        Method called by the timer at a set rate (self.DT)
        '''
        # Restart the timer
        self.timer_is_running = False
        self.start()
        # Call the simulation method
        self.sim()

    def start(self):
        if (not self.timer_is_running):
            self.timer = thrd.Timer(self.DT, self.run)
            self.timer.daemon = True
            self.timer.start()
            self.timer_is_running = True

    def stop(self):
        self.timer.cancel()
        self.timer_is_running = False
        
    def distangle2wall(self,wall):
        '''
        Calculate distance to wall
        Returns: tuple
            d2line - distance from Create to line [m]
            angle2line - angle from Create to line [rad]
        '''
        # slope of the wall
        if (abs(wall.x2-wall.x1) < 0.1):
            m = 10000.0
        else:
            m = float((wall.y2-wall.y1)/(wall.x2-wall.x1))
        # find intercept point
        if abs(m) < 0.1:
            xint = self.X
            yint = wall.y1
        elif abs(m) > 1000.:
            xint = wall.x1
            yint = self.Y
        else:
            xint = (m/(m**2+1))*(self.Y-wall.y1+m*wall.x1+self.X/m)
            #xint = (self.Y-wall.y1+m*wall.x1+self.X/m)/(m+1.0/m)
            yint = wall.y1+m*(xint-wall.x1)
        # make sure that the intersection is within the line
        if (xint <= max(wall.x2, wall.x1) and 
            xint >= min(wall.x1, wall.x2) and
            yint <= max(wall.y2, wall.y1) and
            yint >= min(wall.y1, wall.y2)):
            d2line = sqrt((self.X-xint)**2+(self.Y-yint)**2)
        else:
            d2line = 1000
        angle2line = atan2(yint-self.Y,xint-self.X)
        #print("m: %4.2f, xint: %4.3f, yint: %4.3f, d2line: %4.3f, ang2line: %4.3f"%(m,xint,yint,d2line,angle2line*180/pi))

        return (d2line, angle2line)
        
    def eval_map(self):
        if self.cmap is not None:
            # If we hit any real walls, stop and activate sensor
            bumpLeft = False
            bumpRight = False
            bumpFront = False
            for wall in self.cmap.walls:
                angleWall = atan2(wall.y2-wall.y1,wall.x2-wall.x1)
                dd,ang = self.distangle2wall(wall)
                # note that ang is reported between -pi and pi
                phi = self.A*pi/180
                if phi > pi:
                    phi = phi-(2*pi)
                # If we are facing into the wall then stop, but allow turn
                if dd <= (self.L*1e-3)/2:
                    vel = (self.vel_right+self.vel_left)/2.0
                    # able between direction to wall and current heading
                    dang = min( abs(ang-phi),
                                2*pi - abs(ang-phi))
                    if (dang < 0.1) and (vel >= 0):
                        # head on and moving toward wall
                        #print("Head on - stopping")
                        self.vel_right = 0
                        self.vel_left = 0
                        bumpRight = True
                        bumpLeft = True
                        bumpFront = True
                    elif (dang < (pi/2)) and (vel >= 0):
                        # towards wall and moving toward wall
                        #print("Toward wall - rotation only")
                        omega = (self.vel_right-self.vel_left)/self.L
                        # make sure it turns into the wall
                        omega = copysign(omega,ang-phi)
                        self.vel_right = 0 #omega*self.L/2
                        self.vel_left = 0 # -omega*self.L/2
                        if (ang-phi) < 0:
                            bumpRight = True
                        else:
                            bumpLeft = True
                    elif (dang < pi/2) and (vel < 0):
                        # head on and moving away from wall
                        #print("Toward wall - moving away - allowed")
                        pass
                    elif (dang >= pi/2) and (vel > 0):
                        # head away and moving away
                        #print("Away - moving away - allowed")
                        #print("ang: %4.2f, phi: %4.2f, dang: %4.2f"%
                        #      (ang, phi, dang))
                        pass
                    elif (dang >= (pi/2)) and (vel <= 0):  
                        # head away and moving toward
                        #print("Away and moving twoard - stop")
                        omega = (self.vel_right-self.vel_left)/self.L
                        self.vel_right = 0
                        self.vel_left =  0
                    else:
                        print "Oops - didn't consider this case!"
                                                    
            self.BumpRight = bumpRight
            self.BumpLeft = bumpLeft
            self.BumpFront = bumpFront
            # If we hit any virtwalls, activate sensor
            
        else:
            pass # no map
            
        
    def sim(self):
        '''
        Simulation step
        '''
        # Evaluate if we hit any obstacles
        self.eval_map()
        
        # Increment the state
        dl = self.vel_left*self.DT   # distance moved by left [mm]
        dr = self.vel_right*self.DT
        dd = (dl+dr)/2.0
        self.X += dd*cos(self.A*pi/180.0)*1e-3
        self.Y += dd*sin(self.A*pi/180.0)*1e-3
        R = self.L*1e-3  # [m] - need to verify this
        da = atan((dr-dl)*1e-3/R)*180.0/pi
        self.A = (self.A+da)%(360.0)  # mod 360
        self.distance += dd
        self.angle += da
        eTime = time.time()-self.T0
        # print debug information every second
        if ((eTime-floor(eTime)) < self.DT):
            logger.debug(("%4.2f: X=%4.2f, Y=%4.2f, "+
                          "A=%4.2f, rvel=%d, lvel: %d")%(eTime,
                                                         self.X,
                                                         self.Y,
                                                         self.A,
                                                         self.vel_right,
                                                         self.vel_left))
    ''' 
    The following methods present an interface equivalent to serial.Serial
    '''
    def read(self,size=1):
        '''
        size - number of bytes to read
        returns: bytes read from the port
        '''
        out = ""
        if size >= len(self.Msg):
            out = self.Msg
            self.Msg = ""
        else:
            out = self.Msg[0:size]
            self.Msg = self.Msg[size+1:]
        return out
            
        
    def write(self,data):
        '''
        data - data to send
        returns: number of bytes written
        '''
        # Validate that the data is correctly formatted
        
        # Check the first byte to determine what command (opcode) is
        # being written
        blist = []
        for dd in data:
            blist.append(ord(dd))
        opcode = blist[0]
        if opcode == 128:
            self.OImode = 'passive'
        elif opcode == 129:
            # change baudrate is meaningless
            pass
        elif opcode == 131:
            self.OImode = 'safe'
        elif opcode == 132:
            self.OImode = 'full'
        elif opcode == 136:
            self._demo(blist)
        elif opcode == 135:
            self._cover()
        elif opcode == 143:
            self._cover_and_dock()
        elif opcode == 134:
            self._spot()
        elif opcode == 137:
            self._drive(blist)
        elif opcode == 145:
            self._drive_direct(blist)
        elif opcode == 139:
            self._leds(blist)
        elif opcode == 140:
            self._song(blist)
        elif opcode == 141:
            self._play_song(blist)
        elif opcode == 142:
            self._sensors(blist)
        elif opcode == 149:
            self._query_list(blist)
        elif opcode == 148:
            self._stream(blist)
        elif opcode == 150:
            self._pause_resume_stream(blist)
        elif opcode in [147, 144, 138, 151, 152, 153, 154, 155, 156, 157, 158]:
            logger.info("Functionality not implemented: opcode=%d"%opcode)
        else:
            logger.info("Unrecognized opcode: %d"%opcode)
        
        
    def flush(self):
        pass
    def flushInput(self):
        pass
    def flushOutput(self):
        pass
    def open(self):
        self.start()
    def close(self):
        self.stop()
    
    '''
    Following are the 'private' methods
    '''
    def _demo(self,data):
        logger.info("Demo functionality not yet implemented")
    def _cover(self):
        logger.info("Cover functionality not yet implemented")
    def _cover_and_dock(self):
        logger.info("Cover and Dock functionality not yet implemented")
    def _cover(self):
        logger.info("Spot functionality not yet implemented")
    def _drive(self,data):
        logger.debug("Drive")
        if not(len(data)==5):
            logger.warning("Drive command should have 5 bytes, received %d"%len(data))
        else:
            # Math for driving in a circle
            vel = pici.bytes2val(data[1],data[2])
            radius = pici.bytes2val(data[3],data[4])
            if radius==32678 or radius==32767:
                # Straight
                self.vel_right = vel
                self.vel_left = vel
            elif radius==-1:
                # turn in place clockwise
                self.vel_right = vel
                self.vel_left = -vel
            elif radius==1 or abs(radius)<1:
                # turn in place counter-clockwise
                self.vel_right = -vel
                self.vel_left = vel
            else:
                # radius = v/omega
                omega = float(vel)/float(radius)  # rad/s
                L = self.L  # distance between wheels in mm
                #R = 65.0/2  # Radius of the wheels
                self.vel_right = vel+omega*L/2  # in mm/s
                self.vel_left =  vel-omega*L/2
            

    def _drive_direct(self,data):
        logger.debug("Drive Direct")
        if not(len(data)==5):
            logger.warning("Direct Drive command should have 5 bytes, recieved %d"%len(data))
        else:
            self.vel_right = pici.bytes2val(data[1],data[2])
            self.vel_left = pici.bytes2val(data[3],data[4])
            
    def _leds(self,data):
        logger.info("Functionality not yet implemented")
    def _song(self,data):
        logger.info("Functionality not yet implemented")
    def _play_song(self,data):
        logger.info("Functionality not yet implemented")
    def _sensors(self,data):
        ''' 
        Get group of sensors
        '''
        if not(len(data)==2):
            logger.warning("Drive command should have 5 bytes, received %d"%len(data))
        else:
            if data[1]==0: #get all sensors
                msg = [0]*26
                # Bump and wheel drops
                bwbyte = 0
                if self.BumpRight:
                    bwbyte += 1
                if self.BumpLeft:
                    bwbyte += 2
                msg[0]=bwbyte
                if self.VirtWall:
                    msg[6]=1
                #print type(self.distance)
                hb,lb = pici.val2bytes(int(self.distance))
                msg[12]=hb
                msg[13]=lb
                hb,lb = pici.val2bytes(int(self.angle))
                msg[14]=hb
                msg[15]=lb
                # Make the message
                for mm in msg:
                    self.Msg += chr(mm)

                
        
    def _query_list(self,data):
        logger.info("Functionality not yet implemented")
    def _stream(self,data):
        logger.info("Functionality not yet implemented")
    def _pause_resume_stream(self,data):
        logger.info("Functionality not yet implemented")


def update_roomba_circle(ax,lock):
    while True:
        lock.acquire()
        ax.figure.canvas.draw()
        lock.release()
        sleep(0.3)
    return
        
