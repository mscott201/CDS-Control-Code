#!/bin/python
# Parameters file for motors
import time
import control
import copy
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
# Implement the default Matplotlib key bindings.
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure
from threading import Event
import can

class Motor:
    def __init__(self):
        self.isR = False # Radial axis uses reversed directions, annoyingly, so flag here
        self.isScrew = False
        self.isPhi = False

        self.MAX_SPEED = [300,300,200] ## 2000 seems to be max steps/sec before encoder falls over
        self.SPEED_FACTOR = [1, 1, 1] # Set this to scale step/encoder count from motor to give O(100) values for PID control
        #self.THRESH_E = 0.0125
        self.THRESH_E = 1

        # Controller initiate
        self.Fs = 5 # global sample rate of 10 Hz
        self.Ts = 1/self.Fs

        # Parameters to control motor and encoder, including readout
        self.LENGTH = 500*3 # size of buffer
        self.E = np.zeros(self.LENGTH) #Distance from current position, Y[0] to intended position V[0]
        self.W = np.zeros(self.LENGTH) #control effort
        self.Y = np.zeros(self.LENGTH) #Encoder position normalised by ENCODER FACTOR
        self.S = np.zeros(self.LENGTH) #Encoder position normalised by ENCODER FACTOR
        self.ENC = np.zeros(self.LENGTH) #encoder value
        self.STP = np.zeros(self.LENGTH) #steps value
        self.V = np.zeros(self.LENGTH) #Position to move to
        self.T = np.linspace(0,500*self.Ts,500)
        self.T = np.tile(self.T, (1,3))
        self.plot_update = 10; #cycles between plot update. 
        self.update_update = 100; #currently when setpoint updates
        self.ENCODER_FACTOR = [1,1,1] # [79.2466, 79.2466, 1748.319] #Sets vertical motion to cm and rotation to degrees (old numbers from WCTE)

        self.isHome = np.zeros(3)

        self.E.shape=(3,500)
        self.W.shape=(3,500)
        self.Y.shape=(3,500)
        self.S.shape=(3,500)
        self.ENC.shape=(3,500)
        self.STP.shape=(3,500)
        self.V.shape=(3,500)
        self.T.shape=(3,500)

        #PID parameters, should work OK for all motors
        self.P = 100
        self.I = 0
        self.If = 0.005 
        self.D = 0
        self.Df = 0.1 # filter on Derivative
        self.s = control.tf([1, 0],[1]) 

        #Plotting variables
        self.ax = []
        self.axs = []
        self.lines = []
        self.strain = []

        #control module parameters, initialise gdc and gpd to NULL
        self.gdc = 0
        self.gpd = 0
        self.gpf = 0.05
        self.gp = self.gpf/(self.s + self.gpf)

        # Max value for y-axis in figure 
        self.yscale = 1000
        self.OFFSET_TOZERO = 1000 # this is the value of the 0 (inmm)-> because the motor doesn't
                                  #deal with negative numbers very well we should probably have a large value here
                                  #and then move about this value with v
        self.u = 0 + self.OFFSET_TOZERO # This is the zero-point of the axis, set as the right limit switch
        self.v = [0, 0, 0]; #this is the variable that we change: to move about the 0 that we have chosen, set at the zero point to avoid moving at startup

        self.rlimit = [0,0,0] # Flag if right limit switch is engaged
        self.llimit = [0,0,0] # Flag if left limit switch is engaged

        self.SETPOINT_REACHED = [False, False, False] # Flag for if motor is at the set position
        self.stopped = [True, True, True] # Flag for if motor is stationary 

        self.Active = 1 # Bool to activate (or deactivate) motor control

        self.ActiveAxes = [0,0,0] # Bool to activate individual axes

        self.InitialisedAxes = [0,0,0] # Bool to track initialisation of individual axes

        self.ser = 0; # Initialise serial bus to NULL

        self.gcd = self.update_ctrl() #s domain transfer functrion
        self.gpd = control.sample_system(self.gp, self.Ts, method='tustin',prewarp_frequency=self.Fs) #discrete time controller tf

        self.debug = False

        self.encoderCount = [0,0,0] # Encoder counter
        self.encoderWrap = [1677719,0,0] # Encoder wrap number


        print("Created Motor Object")

    def GetIsHome(self, axis):
        return self.isHome[axis]

    def update_ctrl(self):
        s2 = control.tf([1, 0],[1])
        g = self.P + (self.I/s2) + self.D*s2/(self.Df*s2 + 1)
        local_gcd = control.sample_system(g, self.Ts, method='tustin',prewarp_frequency=self.Fs) # works and matches MATLAB value
        return local_gcd

    def Reset(self):
        RESP = self.MOTOR_COMMAND(self.ser,1,255,0,0,0)#Reset board
        
    # Initialise motor axis
    def INIT_AXIS(self, serial, inNum, axis):
        self.ser = serial

#        if(axis == "R" or axis == "SCREW"):
#            RESP = self.MOTOR_COMMAND_STRING(self.ser,1,136,0, inNum,0)#Get version of firmware
#            if("1180" not in RESP):
#                print("R/Screw axis not connected to 1180 board - check USB port name and re-initialise")
#                exit()
#        else:
#            RESP = self.MOTOR_COMMAND_STRING(self.ser,1,136,0, inNum,0)#Get version of firmware
#            if("3110" not in RESP):
#                print("R axis not connected to 1180 board - check USB port name and re-initialise")
#                exit()

        if(axis == "Z"):
            RESP = self.MOTOR_COMMAND(self.ser,1,5,140, inNum,8)#Set number of microsteps per full step, using recommended value
            RESP = self.MOTOR_COMMAND(self.ser,1,5,13,  inNum,0)#Enable left limit switch
            RESP = self.MOTOR_COMMAND(self.ser,1,5,12,  inNum,0)#Enable (0)  or disable (1) right limit switch
            RESP = self.MOTOR_COMMAND(self.ser,1,5,6,   inNum,100)#set drive current
            RESP = self.MOTOR_COMMAND(self.ser,1,5,5,   inNum,100)#set max acceleration
            RESP = self.MOTOR_COMMAND(self.ser,1,5,201, inNum,1)#reverse encoder direction
            RESP = self.MOTOR_COMMAND(self.ser,1,5,174, inNum,1)#set stall threshold to +1
            RESP = self.MOTOR_COMMAND(self.ser,1,5,173, inNum,1)#set stall filteer to on
            RESP = self.MOTOR_COMMAND(self.ser,1,5,210, inNum,65536)#set encoder prescale to 1
            self.ENCODER_LAG = 0.0

        elif(axis == "R"):
            RESP = self.MOTOR_COMMAND(self.ser,1,5,140, inNum,8)#Set number of microsteps per full step, using recommended value
            RESP = self.MOTOR_COMMAND(self.ser,1,5,13,  inNum,0)#Enable left limit switch
            RESP = self.MOTOR_COMMAND(self.ser,1,5,12,  inNum,0)#Enable (0)  or disable (1) right limit switch
            RESP = self.MOTOR_COMMAND(self.ser,1,5,6,   inNum,96)#set drive current
            RESP = self.MOTOR_COMMAND(self.ser,1,5,7,   inNum,56)#set standby current to avoid motor slipping
            RESP = self.MOTOR_COMMAND(self.ser,1,5,5,   inNum,100)#set max acceleration
            RESP = self.MOTOR_COMMAND(self.ser,1,5,174, inNum,4)#set stall threshold to +4
            RESP = self.MOTOR_COMMAND(self.ser,1,5,173, inNum,1)#set stall filteer to on
            RESP = self.MOTOR_COMMAND(self.ser,1,9,74,  inNum,0)#set to use external encoder (only the 1180 board)
            RESP = self.MOTOR_COMMAND(self.ser,1,14,0,  inNum,0)#set limit switch to no pull up (for proximity limit switch)
            RESP = self.MOTOR_COMMAND(self.ser,1,5,210, inNum,512)#set encoder prescale to 1

            self.ENCODER_FACTOR[0] = 8965.0331
            self.ENCODER_LAG = 0.0 #Measured lag roughly.
            self.MAX_SPEED[0] = 200  
            self.isR = True

        elif(axis == "SCREW"):
            RESP = self.MOTOR_COMMAND(self.ser,1,5,140, inNum,8)#Set number of microsteps per full step, using recommended value
            RESP = self.MOTOR_COMMAND(self.ser,1,5,13,  inNum,0)#Enable left limit switch
            RESP = self.MOTOR_COMMAND(self.ser,1,5,12,  inNum,0)#Enable (0)  or disable (1) right limit switch
            RESP = self.MOTOR_COMMAND(self.ser,1,5,6,   inNum,96)#set drive current
            RESP = self.MOTOR_COMMAND(self.ser,1,5,5,   inNum,100)#set max acceleration
#            RESP = self.MOTOR_COMMAND(self.ser,1,5,201, inNum,0)#reverse encoder direction
            RESP = self.MOTOR_COMMAND(self.ser,1,5,174, inNum,4)#set stall threshold to +4
            RESP = self.MOTOR_COMMAND(self.ser,1,5,173, inNum,1)#set stall filteer to on
            RESP = self.MOTOR_COMMAND(self.ser,1,5,210, inNum,65536)#set encoder prescale to 1

            self.ENCODER_FACTOR[0] = 57.36
            self.ENCODER_LAG = 0.0 #Measured lag roughly.
            self.MAX_SPEED[0] = 1000 
            self.SPEED_FACTOR[0] = 10
            self.isScrew = True

        elif(axis == "PHI"):
            RESP = self.MOTOR_COMMAND(self.ser,1,5,140, inNum,8)#Set number of microsteps per full step, using recommended value
            RESP = self.MOTOR_COMMAND(self.ser,1,5,13,  inNum,0)#Enable left limit switch
            RESP = self.MOTOR_COMMAND(self.ser,1,5,12,  inNum,1)#Enable (0)  or disable (1) right limit switch
            RESP = self.MOTOR_COMMAND(self.ser,1,5,6,   inNum,80)#set drive current
            RESP = self.MOTOR_COMMAND(self.ser,1,5,5,   inNum,100)#set max acceleration
            RESP = self.MOTOR_COMMAND(self.ser,1,5,174, inNum,12)#set stall threshold to +12
            RESP = self.MOTOR_COMMAND(self.ser,1,5,173, inNum,1)#set stall filter to on
            RESP = self.MOTOR_COMMAND(self.ser,1,5,201, inNum,0)
            RESP = self.MOTOR_COMMAND(self.ser,1,5,210, inNum,65536)#set encoder prescale to 1
            self.isPhi = True
            self.ENCODER_LAG = 0.0
            self.ENCODER_FACTOR[1] = 50.0
            self.SPEED_FACTOR[1] = 10
            self.MAX_SPEED[1] = 500 

        else:
            print("Please input a recognised axis: Z, R, PHI")
            return 1
        
        # Deactivate system so that the motors cannot move
        self.disable_axis(inNum)
        self.InitialisedAxes[inNum] = 1
        print("Initialised axis number = ", inNum)

        return 0


    def build_tx(self, COMMAND,TYPE,MOTOR,VALUE):
        TX = bytearray(3)
        TX[0] = COMMAND # command number
        TX[1] = TYPE # type number
        TX[2] = MOTOR # motor
        TX[3:3] = bytearray(VALUE.to_bytes(4,byteorder='big',signed=True))
    
        return TX
    
    # Method to send commands to motor controller board and get response - ADD kept for compatibility with USB protocol, but not needed
    def MOTOR_COMMAND(self,DEV,ADD,CMD,TYPE,MOTOR,VALUE):
        TX = self.build_tx(CMD, TYPE, MOTOR, VALUE)
        sff_frame = can.Message(arbitration_id=0x001, data=TX, is_extended_id=False)
        DEV.send(sff_frame)
        msg = DEV.recv(10.0)
        RESP = int.from_bytes(msg.data[3:7], "big", signed=True)

#        if(CMD==6 and TYPE==3):
#            print("TX  {}".format(sff_frame))
#            print("RX  {}".format(msg))
#            print("RESP = ", RESP) 

        return RESP
    
    # Method to send commands to motor controller board and get response - ADD kept for compatibility with USB protocol, but not needed
    def MOTOR_COMMAND_STRING(self,DEV,ADD,CMD,TYPE,MOTOR,VALUE):
        TX = self.build_tx(CMD, TYPE, MOTOR, VALUE)
        sff_frame = can.Message(arbitration_id=0x001, data=TX, is_extended_id=False)
#        print("TX  {}".format(sff_frame))
        DEV.send(sff_frame)
        msg = DEV.recv(10.0)
#        print("RX  {}".format(msg))
    
        return msg.data.decode(encoding='utf-8', errors='strict')
    
    # Evaluate the work effort needed to move from current position to desired position
    def DE_EVAL(self, axis): #requires a control.tf object in discrete form.
        NUM = self.gcd.num[0][0]
        DEN = self.gcd.den[0][0]
        out = 0.0
        for i in range(0,len(NUM)):
            out = out + NUM[i]*self.E[axis][i]        
        for j in range(1,len(DEN)):
            out = out - DEN[j]*self.W[axis][j]

        return out                   
    
    def DE_SHIFT(self, ARY): #shifts all values down and clears initial value, for control algorithm 
        ARY[1:] = ARY[0:-1]
        ARY[-1] = 0
        return ARY

    # Initialise the motor controller options
    # Should customise this for each axis
    # Send the motor to the right limit switch then zero the encoder and step positions
    def SendToHome(self, event, axis):

        self.isHome[axis] = 0

        #Re-enable both limit switches
        if not (self.isPhi):
            RESP = self.MOTOR_COMMAND(self.ser,1,5,12, axis,0)#Enable right limit switch
        RESP = self.MOTOR_COMMAND(self.ser,1,5,13, axis,0)#Enable (0)  or disable (1) left limit switch
        speed = self.MAX_SPEED[axis]

        # Move Left until left limit switch is hit
        RESP = self.MOTOR_COMMAND(self.ser,1,2,0, axis,speed) # ROL at +ve/-ve velocity, drive slowly back to limit
        self.stopped[axis] = False
        self.Active = True 
        print("Starting home procedure")
        print("Active = ", self.Active)
        while self.Active:
            SPEED = self.MOTOR_COMMAND(self.ser,1,6,3, axis,0) # get speed
            #Stop the motor when the limit switch has been hit
            if SPEED == 0:
                self.Active = False
                RESP = self.MOTOR_COMMAND(self.ser,1,3,0, axis,200) #stop
                self.stopped[axis] = True

            self.STP[axis][0] = self.MOTOR_COMMAND(self.ser,1,6,1,axis,0) # get steps
            self.STP[axis][0] = float(self.STP[axis][0])/256 # scale by no. of microsteps 
            self.ENC[axis][0] = self.MOTOR_COMMAND(self.ser,1,6,209,axis,0) # get encoder position
            self.S[axis][0] = self.MOTOR_COMMAND(self.ser,1,6,206,axis,0) # get stall value

            # Calculate encoder position in lab, should be OFFSET_TOZERO if at right limit switch
            self.updateEncoderPosition(axis)

            # Calculate encoder position in lab, should be OFFSET_TOZERO if at right limit switch
            self.Y[axis][0] = self.STP[axis][0]/self.ENCODER_FACTOR[axis]
 
            self.Y[axis] = np.roll(self.Y[axis],1)
            self.S[axis] = np.roll(self.S[axis],1)
            self.E[axis] = np.roll(self.E[axis],1)
            self.W[axis] = np.roll(self.W[axis],1)
            self.V[axis] = np.roll(self.V[axis],1)
            self.STP[axis] = np.roll(self.STP[axis],1)
            self.ENC[axis] = np.roll(self.ENC[axis],1)

            self.Y[axis][0] = self.Y[axis][1]
            self.S[axis][0] = self.S[axis][1]
            self.E[axis][0] = self.E[axis][1]
            self.W[axis][0] = self.W[axis][1]
            self.V[axis][0] = self.V[axis][1]
            self.STP[axis][0] = self.STP[axis][1]
            self.ENC[axis][0] = self.ENC[axis][1]

            time.sleep(0.1) 

        print("Finished motion")

        #Re-enable both limit switches
        if not(self.isPhi):
            RESP = self.MOTOR_COMMAND(self.ser,1,5,12, axis,0)#Enable right limit switch
        RESP = self.MOTOR_COMMAND(self.ser,1,5,13, axis,0)#Enable (0)  or disable (1) left limit switch

        #Get status of limit switches
        if self.isR:
            self.llimit[axis] = self.MOTOR_COMMAND(self.ser,1,6,10,axis,0) # get right limit switch state
            self.rlimit[axis] = self.MOTOR_COMMAND(self.ser,1,6,11,axis,0) # get left limit switch state
        else:
            self.llimit[axis] = self.MOTOR_COMMAND(self.ser,1,6,11,axis,0) # get left limit switch state
            self.rlimit[axis] = self.MOTOR_COMMAND(self.ser,1,6,10,axis,0) # get right limit switch state

        # Zero all positions and encoders
        self.zero_position_variables(axis)
        self.set_active()
        self.enable_axis(axis)
        print("Axis", axis," has arrived at home")
        self.isHome[axis] = 1

        event.set()

        return 0

    def get_left_limit_status(self, axis):
        return self.llimit[axis]

    def get_right_limit_status(self, axis):
        return self.rlimit[axis]

    def set_target_position(self, pos, axis):
        pos2 = abs(pos)*self.ENCODER_FACTOR[axis] + self.ENCODER_LAG 
        pos2 = math.copysign(pos2, pos)
        print("vpos = ", pos2)
        self.v[axis] = pos2;
        return pos

    def STOP(self):
        for i in range(3):
            self.stop_axis(i)
        self.set_inactive()
        return 0

    def stop_axis(self, axis):
        if self.InitialisedAxes[axis]:
            RESP = self.MOTOR_COMMAND(self.ser,1,3,0,axis,50) #stop

        self.stopped[axis] = True
        return 0

    def set_active(self):
        self.Active = 1;
        return 0

    def set_inactive(self):
        self.Active = 0;
        self.ActiveAxes = [0,0,0]
        return 0

    def enable_axis(self, axis):
        self.ActiveAxes[axis] = 1
        return 0

    def disable_axis(self, axis):
        self.ActiveAxes[axis] = 0
        return 0

    def get_step_position(self, axis):
        return ((self.STP[axis][0] - self.OFFSET_TOZERO)/self.ENCODER_FACTOR[axis])

    def get_encoder_position(self, axis):
        pos = self.Y[axis][0]
        return pos

    def get_stopped(self, axis):
        return self.stopped[axis]

    def zero_position_variables(self, axis):
        # Reset all movement variables
        self.u = 0 + self.OFFSET_TOZERO # This is the zero-point of the axis, set as the right limit switch
        self.v[axis] = 0; #this is the variable that we change: to move about the 0 that we have chosen, set at the zero point to avoid moving at startup
        for i in range(len(self.V[axis])):
            self.V[axis][i] = 0
            self.Y[axis][i] = 0
            self.S[axis][i] = 900
            self.STP[axis][i] = self.u
            self.ENC[axis][i] = self.u*self.ENCODER_FACTOR[axis]
            self.E[axis][i] = 0
            self.W[axis][i] = 0
        # Reset positions on board to ensure they match what is displayed on the GUI
        RESP = self.MOTOR_COMMAND(self.ser,1,5,209, axis, int(self.ENC[axis][0]) ) # set encoder position as offset * factor
        RESP = self.MOTOR_COMMAND(self.ser,1,5,1, axis, int(self.STP[axis][0])*256) # set steps position as offset * microsteps/step

        self.encoderCount[axis] = 0

    def checkStall(self, axis, SPEED):
        # Using numbers for Z axis motion for now
        # Need to tune for other axes and after final speed tuning
        if abs(SPEED) > 100 and self.S[axis][0] < 100:
            self.stop_axis(axis)
            return True
        return False

    def updateEncoderPosition(self, axis):
        # Update internal encoder counter to avoid wrapping issues
        diff = self.ENC[axis][0] - self.ENC[axis][1]
        if abs(diff) > 10000:
            if abs(diff) > self.encoderWrap[axis] and abs(self.ENC[axis][1]) > 1:
                self.encoderWrap[axis] = abs(diff)

            diff = self.encoderWrap[axis] - abs(diff)

        self.encoderCount[axis] += diff


    def CONTROLLER_TICK(self):
        next_call = time.time()
        
        while True:
            # Check whether axes should be moving
            axes_to_move = []
            for i in range(3):
                if self.ActiveAxes[i] == 1:
                    axes_to_move.append(i)
                else:
                    # Get current motor speed
                    SPEED = self.MOTOR_COMMAND(self.ser,1,6,3, i,0) # get speed
                    if abs(SPEED) > self.THRESH_E:
                        self.stop_axis(i)

            if(self.debug):
                print("Active axes = ", self.ActiveAxes)
                print("axes list = ", axes_to_move)

            #If the active switch is set to false quit the control program, stopping the thread
            if(self.Active == False):
                return 0

            for axis in axes_to_move:
                self.STP[axis][0] = self.MOTOR_COMMAND(self.ser,1,6,1,axis,0) # get steps
                self.STP[axis][0] = float(self.STP[axis][0])/256 # Divide by number of microsteps to give actual step count
                self.ENC[axis][0] = self.MOTOR_COMMAND(self.ser,1,6,209,axis,0) # get encoder position
                self.S[axis][0] = self.MOTOR_COMMAND(self.ser,1,6,206,axis,0) # get motor axis strain

                self.updateEncoderPosition(axis)

                # Calculate encoder position in lab 
                self.Y[axis][0] = (self.STP[axis][0] - self.OFFSET_TOZERO)
                self.V[axis][0] = self.v[axis] # Position to travel to, plus zero offset (u)

                #v -> position from the 0, defined as the right limit switch.
                self.E[axis][0] = self.Y[axis][0] - self.V[axis][0] #error function, distance 

                self.E[axis][0] = self.E[axis][0] / self.SPEED_FACTOR[axis]

                if(abs(self.E[axis][0]) < self.THRESH_E): #deadband
                    self.SETPOINT_REACHED[axis] = True
                    self.E[axis][0] = 0
                else:
                    self.SETPOINT_REACHED[axis] = False

                self.W[axis][0] = self.DE_EVAL(axis) #evaluate control effort
                SPEED = self.MOTOR_COMMAND(self.ser,1,6,3, axis,0) # get speed

                stall = self.checkStall(axis, SPEED)
                if(self.debug):
                    print("Step position = ", self.STP[axis][0])
                    print("Encoder readout value = ", self.ENC[axis][0])
                    print("Encoder count value = ", self.encoderCount[axis])
                    print("self.Y[axis][0] = ",self.Y[axis][0])
                    print("self.S[axis][0] = ",self.S[axis][0])
                    print("self.V[axis][0] = ", self.V[axis][0])
                    print("SELF.E[axis][0] = ", self.E[axis][0])
                    print("W[axis][0] =", self.W[axis][0])
                    print("Strain = ", self.S[axis][0])
                    print("SPEED = ", SPEED)
                    print("stall = ", stall)

                #if stall:
                    #print("Stopped due to stall")
                    #self.v[axis] = self.Y[axis][0]
                    #self.stop_axis(axis)  

                # Get current motor speed
                if(abs(SPEED) > self.THRESH_E):
                    self.stopped[axis] = False
                else:
                    self.stopped[axis] = True


                # Check if work is very small, i.e. reached position     
                if( abs(self.W[axis][0]) < 1):
                    EFF = 0 
                    # If moving, stop
                    if(abs(SPEED) > self.THRESH_E):
                        self.stop_axis(axis)

                # If work is not small and positive, move with speed = work or max speed
                else: # abs(self.W[axis][0]) > 0 :
                    EFF = min(abs(self.W[axis][0]), self.MAX_SPEED[axis])
                    EFF = math.copysign(EFF, self.W[axis][0])
                    RESP = self.MOTOR_COMMAND(self.ser,1,2,0,axis,int(EFF)) # ROL at velocity EFF

                #Get status of limit switches
                if self.isR:
                    self.llimit[axis] = self.MOTOR_COMMAND(self.ser,1,6,10,axis,0) # get right limit switch state
                    self.rlimit[axis] = self.MOTOR_COMMAND(self.ser,1,6,11,axis,0) # get left limit switch state
                else:
                    self.llimit[axis] = self.MOTOR_COMMAND(self.ser,1,6,11,axis,0) # get left limit switch state
                    self.rlimit[axis] = self.MOTOR_COMMAND(self.ser,1,6,10,axis,0) # get right limit switch state



                #thread handling
                time.sleep(0.01) #00

            # Move the current value of the parameters back one in the list, to allow plotting
            # But, copy current value to front of list, to ensure it is picked up by GUI
            # Do this even when not moving axes
            for i in range(3):
                self.Y[i] = np.roll(self.Y[i],1)
                self.S[i] = np.roll(self.S[i],1)
                self.E[i] = np.roll(self.E[i],1)
                self.W[i] = np.roll(self.W[i],1)
                self.V[i] = np.roll(self.V[i],1)
                self.STP[i] = np.roll(self.STP[i],1)
                self.ENC[i] = np.roll(self.ENC[i],1)
                    
                self.Y[i][0] = self.Y[i][1]
                self.S[i][0] = self.S[i][1]
                self.E[i][0] = self.E[i][1]
                self.W[i][0] = self.W[i][1]
                self.V[i][0] = self.V[i][1]
                self.STP[i][0] = self.STP[i][1]
                self.ENC[i][0] = self.ENC[i][1]


            next_call = next_call+self.Ts;
            time.sleep(max((next_call - time.time())-(len(axes_to_move)*0.01),0.001)) #00

        return

    def isSetPointReached(self, axis):
        return self.SETPOINT_REACHED[axis]

    def SetPlot(self, axes, strains):
        for i in range(len(axes)):
            self.ax.append(axes[i])
            line = self.ax[i].plot(self.T[i][1:], (np.flip(self.Y[i][:-1])/(self.ENCODER_FACTOR[i])), color="red", lw=4) # was 1
            self.lines.append(line[0])

        for i in range(len(strains)):
            self.axs.append(strains[i])
            line = self.axs[i].plot(self.T[i][1:], np.flip(self.S[i][:-1]), color="blue", lw=4) # was 1
            self.strain.append(line[0])


        return 0

    def animate(self):
        # function to update figures
        for i in range(len(self.lines)):
            self.lines[i].set_ydata(np.flip(self.Y[i][:-1]/(self.ENCODER_FACTOR[i])))
        for i in range(len(self.strain)):
            self.strain[i].set_ydata(np.flip(self.S[i][:-1]))

        linelist = copy.deepcopy(self.lines)
        linelist.extend(self.strain)

        return copy.deepcopy(linelist)

