#IMPORTS######################################################################
import threading
from CDS_Motor_PlusStrain import Motor
from CDS_Laser import Laser
from tkinter import *
from tkinter import ttk
from tkinter import filedialog
from functools import partial
from matplotlib.backend_bases import key_press_handler
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure
import numpy as np
import tkinter
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import time
from scipy import interpolate
import usb.core
import usb.util
import sys


#%% Initiate motor drive
# Axes in this class are labelled from 1 to 4.
# Axes in CDS_Motor are labelled from 0 to 2, since each card has max 3 axes and they start at 0.
class CDS_Control():

    def __init__(self, root):

        self.move_event = -1

        self.laser = -1
        self.trig = 1

        self.current_state = []

        self.style = ttk.Style()
        self.style.theme_use('alt')

        self.style.configure("error.TLabel", foreground='white', background='red')
        self.style.configure("LD.TButton", foreground='white', background='green')
        self.style.configure("TEC.TButton", foreground='white', background='green')
        self.style.configure("EXT.TButton", foreground='black', background='white')

        self.root = root

        self.root.title("CDS Control Interface")

        self.motor1 = Motor()
        self.motor2 = Motor()

        self.hasMotor1 = 0
        self.hasMotor2 = 0

        self.ControlThread1 = 0
        self.ControlThread2 = 0
        self.LoopThread = 0

        self.hasMotor1Axis1 = 0
        self.hasMotor1Axis2 = 0
        self.hasMotor1Axis3 = 0

        self.hasMotor2Axis1 = 0

        self.homed = [0,0,0,0]

        self.mainframe = ttk.Frame(self.root)#, padding="3 3 15 15")
        self.mainframe.grid(column=0, row=0, sticky=(N, W, E, S))
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        self.mainframe.bind("<Configure>", self.dosize)

        self.Port1 = StringVar(value = '/dev/ttyACM0')
        ttk.Entry(self.mainframe, textvariable=self.Port1).grid(column=2, row=0,columnspan=2, sticky=(W, E))
        ttk.Label(self.mainframe, text="Motor Controller 1 COM port:").grid(column=0, row=0,columnspan=2, sticky=W)

        self.btnSerial1 = ttk.Button(self.mainframe, text="Open Controller 1 Serial Port",command=self.openSerial1)
        self.btnSerial1.grid(row=1,column=0,columnspan=2,pady=20, sticky=(N, W, E, S))
        self.btnSerClo1 = ttk.Button(self.mainframe, text="Close Controller 1 Serial Port",command=self.closeSerial1)
        self.btnSerClo1.grid(row=1,column=2,columnspan=2,pady=20, sticky=(N, W, E, S))

#        self.mainframe.rowconfigure(2, minsize=30)

        btnMotor1 = ttk.Button(self.mainframe, text="Initialise Axis 1",command=self.InitialiseAxis1).grid(row=3,column=0,columnspan=2, sticky=(N, W, E, S))

        self.btnHome1 = ttk.Button(self.mainframe, text="Axis 1 Return to Home",command=partial(self.home, 1))
        self.btnHome1.grid(row=4,column=0,columnspan=2, sticky=(N, W, E, S))

        self.PosEnc1 = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.PosEnc1).grid(column=1, row=5, sticky=(W, E))
        ttk.Label(self.mainframe, text="Axis 1 Encoder position:").grid(column=0, row=5, sticky=W)
 
        self.PosStep1 = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.PosStep1).grid(column=1, row=6, sticky=(W, E))
        ttk.Label(self.mainframe, text="Axis 1 step position:").grid(column=0, row=6, sticky=W)

        self.Target1 = StringVar(value='0')
        ttk.Entry(self.mainframe, textvariable=self.Target1).grid(column=1, row=7, sticky=(W, E))
        ttk.Label(self.mainframe, text="Axis 1 target:").grid(column=0, row=7, sticky=W)

        self.btnStart1 = ttk.Button(self.mainframe, text="Start Controller 1 Program",command=partial(self.start, 1))
        self.btnStart1.grid(row=9,column=0,columnspan=6, sticky=(N, W, E, S))
        self.btnStpP1 = ttk.Button(self.mainframe, text="Stop Controller 1 Program",command=partial(self.stop, 1))
        self.btnStpP1.grid(row=10,column=0,columnspan=6, sticky=(N, W, E, S))

        btnMove1 = ttk.Button(self.mainframe, text="Axis 1 Move to Position",command=partial(self.singlemove, 1)).grid(row=12,column=0,columnspan=2, sticky=(N, W, E, S))
        btnStop1  = ttk.Button(self.mainframe, text="Stop Axis 1",command=partial(self.stopaxis, 1)).grid(row=13,column=0,columnspan=2, sticky=(N, W, E, S))

        self.btnStopAll  = ttk.Button(self.mainframe, text="Stop All Axes",command=self.stopall)
        self.btnStopAll.grid(row=14,column=0,columnspan=8, sticky=(N, W, E, S))

        button = ttk.Button(master=self.mainframe, text="Quit", command=self.__del__).grid(row=15,column=0,columnspan=8, sticky=(N, W, E, S))

        self.fig1 = Figure(figsize=(9, 4), dpi=self.root.winfo_fpixels('1i'))
        ax1 = self.fig1.add_subplot(1,2,1)
        ax2 = self.fig1.add_subplot(1,2,2)
        ax = [ax1, ax2]
 
        for index in range(2):
            if(index == 0):
                ax[index].set_ylim(-10, 300)
                ax[index].set_ylabel("Arm Angle / degrees")
            else:
                ax[index].set_ylim(-10,1000)
                ax[index].set_ylabel("1/Motor Load")

            ax[index].set_xlim(0, 100)
            ax[index].set_xlabel("Time / s")
            ax[index].grid()

        self.SetPlots(ax1, ax2)
        self.fig1.tight_layout()
        self.canvas1 = FigureCanvasTkAgg(self.fig1, master=self.root)  # A tk.DrawingArea.
        self.canvas1.draw()
        toolbar1 = NavigationToolbar2Tk(self.canvas1, self.root, pack_toolbar=False)
        toolbar1.update()
        self.canvas1.mpl_connect("key_press_event", lambda event: print(f"you pressed {event.key}"))
        self.canvas1.mpl_connect("key_press_event", key_press_handler)
        self.canvas1.get_tk_widget().grid(row=16,column=0,columnspan=8, sticky=(N, S, E, W))

        self.ani = animation.FuncAnimation(self.fig1, self.anim, interval=50, blit=False, save_count=10)

        for child in self.mainframe.winfo_children(): 
            child.grid_configure(padx=5, pady=5)


        self.Waittime = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.Waittime, width=15).grid(column=9, row=9, sticky=(W))
        self.WaittimeLabel = ttk.Label(self.mainframe, text="Wait time:")
        self.WaittimeLabel.grid(column=8, row=9, sticky=E)

        self.Angle = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.Angle, width=15).grid(column=9, row=11, sticky=(W))
        self.AngleLabel = ttk.Label(self.mainframe, text="Angle:")
        self.AngleLabel.grid(column=8, row=11, sticky=E)

        self.btnAutoMove  = ttk.Button(self.mainframe, text="Start Auto Movement",command=self.automove)
        self.btnAutoMove.grid(row=13,column=8,columnspan=2, sticky=(N, W, E, S))
        self.btnMovePosition  = ttk.Button(self.mainframe, text="Next Position",command=self.next_position)
        self.btnMovePosition.grid(row=14,column=8,columnspan=2, sticky=(N, W, E, S))

        self.mainframe.columnconfigure(10, minsize=10, weight = 0) 

        self.connectLaser = ttk.Button(self.mainframe, text="Connect to Laser",command=self.connectToLaser)
        self.connectLaser.grid(row=0,column=11,columnspan=2, sticky=(N, W, E, S))

        self.ldcurrent = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.ldcurrent, width=15).grid(column=12, row=2, sticky=(W))
        self.ldcurrentLabel = ttk.Label(self.mainframe, text="LD Current (mA)")
        self.ldcurrentLabel.grid(column=11, row=2, sticky=E)
        self.btnldcurrent = ttk.Button(self.mainframe, text="Set LD Current",command=self.setLDCurrent)
        self.btnldcurrent.grid(row=3,column=11,columnspan=1, sticky=(N, W, E, S))
        self.ldcurrentIn = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.ldcurrentIn, width=15).grid(column=12, row=3, sticky=(W))

        self.trigrate = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.trigrate, width=15).grid(column=12, row=5, sticky=(W))
        self.trigRateLabel = ttk.Label(self.mainframe, text="Trigger Rate (kHz)")
        self.trigRateLabel.grid(column=11, row=5, sticky=E)
        self.btntrigrate = ttk.Button(self.mainframe, text="Set Trig Rate",command=self.setTriggerRate)
        self.btntrigrate.grid(row=6,column=11,columnspan=1, sticky=(N, W, E, S))
        self.trigrateIn = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.trigrateIn, width=15).grid(column=12, row=6, sticky=(W))

        self.pulsewidth = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.pulsewidth, width=15).grid(column=12, row=8, sticky=(W))
        self.pulseWidthLabel = ttk.Label(self.mainframe, text="Pulse Width (ps)")
        self.pulseWidthLabel.grid(column=11, row=8, sticky=E)
        self.btnpulsewidth = ttk.Button(self.mainframe, text="Set Pulse Width",command=self.setPulseWidth)
        self.btnpulsewidth.grid(row=9,column=11,columnspan=1, sticky=(N, W, E, S))
        self.pulsewidthIn = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.pulsewidthIn, width=15).grid(column=12, row=9, sticky=(W))

        self.laserPDCurrent = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.laserPDCurrent, width=15).grid(column=12, row=11, sticky=(W))
        self.laserPDCurrentLabel = ttk.Label(self.mainframe, text="PD Current (pA)")
        self.laserPDCurrentLabel.grid(column=11, row=11, sticky=E)

        self.btnldstatus = ttk.Button(self.mainframe, text="Switch LD ON", command=self.setLDStatus, style="LD.TButton")
        self.btnldstatus.grid(row=13,column=11,columnspan=2, sticky=(N, W, E, S))

        self.btntecstatus = ttk.Button(self.mainframe, text="Switch TEC ON",command=self.setTECStatus, style="TEC.TButton")
        self.btntecstatus.grid(row=14,column=11,columnspan=2, sticky=(N, W, E, S))

        self.btnexttrig = ttk.Button(self.mainframe, text="Switch EXT ON",command=self.setExtTrig, style="EXT.TButton")
        self.btnexttrig.grid(row=15,column=11,columnspan=2, sticky=(N, W, E, S))


        self.mainframe.rowconfigure(16, weight=1)

        self.radius = -1
        self.angle = 0
        self.zpos = -10

        return 
 
    def anim(self, i):
        # function to update figures
        lines = []

        if(self.hasMotor1):
            lines = self.motor1.animate()

        return lines

    def dosize(self, eve):
        wi = root.winfo_width()
        dpi=self.root.winfo_fpixels('1i')
        x, y, w, hi = self.mainframe.grid_bbox(row=16)
        #print(wi, hi)
        hi = hi*0.8
        self.canvas1.get_tk_widget().configure(width=wi, height=hi)
        self.fig1.set_size_inches(w=wi/dpi, h=hi/dpi, forward=True)
        self.canvas1.draw()

    def openSerial1(self):
        inPort = self.Port1.get()
        self.ser1 = 0
        self.ser1 = serial.Serial(
                    port=inPort,
                    baudrate=115200,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    timeout=2
        )
        print("Serial Port",inPort, "has been opened")
        self.hasMotor1 = 1;
        self.btnSerial1["state"] = "disabled"
        self.btnSerClo1["state"] = "normal"
        return 0

    def closeSerial1(self):
        if(self.hasMotor1):
            self.stop(1)
        if(self.ser1.isOpen()):
            self.ser1.close()
        print("Serial Port 1 has been closed")
        self.hasMotor1 = 0;
        self.btnSerClo1["state"] = "disabled"
        self.btnSerial1["state"] = "normal"
        return 0

    def InitialiseAxis1(self):
        axis = 0
        port = self.Port1.get()
        if(not self.hasMotor1):
            print("Cannot initialise axis without opening serial port")
            return 1

        self.motor1.INIT_AXIS(self.ser1, port, axis, 'Z')
        self.hasMotor1Axis1 = 1
        return 0

    def UpdatePos(self):
        if(self.hasMotor1Axis1):
            self.PosEnc1.set(str(self.motor1.get_encoder_position(0)))
            self.PosStep1.set(str(self.motor1.get_step_position(0)))

            left = self.motor1.get_left_limit_status(0)
            right = self.motor1.get_right_limit_status(0)
            if(left):
                self.Axis1LimitLeftLabel.configure(style="error.TLabel")
            else:
                self.Axis1LimitLeftLabel.configure(style="TLabel")
            if(right):
                self.Axis1LimitRightLabel.configure(style="error.TLabel")
            else:
                self.Axis1LimitRightLabel.configure(style="TLabel")

            self.Axis1LimitLeft.set(str(left))
            self.Axis1LimitRight.set(str(right))

        if(self.laser != -1):
            self.laserPDCurrent.set(str(self.laser.GetPDCurrent()))
            self.pulsewidth.set(str(self.laser.GetPulseWidth()*10))
            self.trigrate.set(str(self.getTriggerRate()))
            self.ldcurrent.set(str(self.laser.GetLDCurrent()))

        self.root.after(500, app.UpdatePos)

        return 0

    def SetPlots(self, ax1, ax2):
        axes = [ax1]
        strains = [ax2]
        self.motor1.SetPlot(axes, strains)
        return 0

    def start(self, motor_nb):
        #this is the main controller thread
        if(self.hasMotor1 and self.ControlThread1 == 0 and motor_nb == 1):
            self.motor1.set_active()
            self.ControlThread1 = threading.Thread(target=self.motor1.CONTROLLER_TICK, name='Motor1')
            self.ControlThread1.start()
            self.btnStpP1["state"] = "normal"
            self.btnStart1["state"] = "disabled"
           
        else:
            print("Cannot start motor program since motor has not been initialised or the control program is already running")
            return 1

        return 0
 
    def stop(self, motor_nb):
        if(self.hasMotor1 and motor_nb == 1):
            for i in range(3):
                self.motor1.disable_axis(i)
            time.sleep(0.25)
            self.motor1.STOP()
            if(self.ControlThread1):
                self.motor1.set_inactive()
                self.ControlThread1 = 0
                self.btnStpP1["state"] = "disabled"
                self.btnStart1["state"] = "normal"
        else:
            print("Trying to stop a controller that has not been initialised")
            return 1
        print("Have stopped controller", motor_nb)
        return 0        

    def stopaxis(self, axis_nb):
        if(self.hasMotor1 and axis_nb < 4):
            self.motor1.disable_axis(axis_nb-1)
        else:
            print("Trying to stop a axis that has not been initialised")
            return 1
        print("Have stopped axis", axis_nb)
        return 0

    def stopall(self):
        self.stop(1)
        return 0

    def singlemove(self, motor_nb):
        if(motor_nb == 1):
            position = float(self.Target1.get())
            return self.move(motor_nb, position)

    def move(self, motor_nb, position):
        if(motor_nb < 4):
            if(not self.hasMotor1):
                print("Please initialise Motor controller 1 to move axis", motor_nb)
                return 1
            elif(not self.ControlThread1):
                print("Please start controller program 1 to move axis", motor_nb)
                return 1

        if(motor_nb < 4):
            self.motor1.enable_axis(motor_nb-1)

        if(motor_nb == 1):
            print("Moving axis 1 to position: ", self.motor1.set_target_position(position,0))

        return 0

    def event_wait_loop(self, event, motor_nb, finished=0):
        if finished:
            self.enable_frame()
            self.homed[motor_nb -1] = 1
            return 0
        else:
            self.root.after(500, self.event_wait_loop, event, motor_nb, event.is_set())

    def home(self, motor_nb):
        if(motor_nb == 1):
            if(self.hasMotor1Axis1):
                self.motor1.set_active()
                event = threading.Event()
                ControlThread = threading.Thread(target=self.motor1.SendToHome, args=(event,0,), name='Motor1Home')
                ControlThread.start()
                self.disable_frame()
                self.event_wait_loop(event,1,0)
                self.zpos = -10
            else:
                print("Cannot send motor home without initialising axis")
                return 1

        print(self.homed)

        return 0

    def disable_frame(self):
        self.current_state = []
        for child in self.mainframe.winfo_children():
            self.current_state.append(child.cget("state"))
            child.configure(state="disabled")
        self.btnStopAll.configure(state="disable")

    def enable_frame(self):
        count = 0
        for child in self.mainframe.winfo_children():
            child.configure(state=self.current_state[count])
            count = count + 1

    def isHomed(self):
        hom = 1
        for home in self.homed:
            hom *= home
        if not hom:
            print("Please send each axis to home before trying to move them.")
        return hom 

    def isInitialised(self):
        initial = (self.hasMotor1Axis1)
        if not initial:
            print("Please initialise all motor axes before using the Automove function")
        return initial

    def rotate(self, phi, event):
        if self.motor1.get_stopped(0):
            print("Moving to angle", phi)
            self.angle = phi
            self.move(1, phi)
            time.sleep(0.5)

        if self.motor1.isSetPointReached(0):
            event.set()
            print("Moved to angle", phi)
        if not event.is_set():
            self.root.after(500, self.rotate, phi, event)

        return 0

    def automove(self):
        self.move_event = threading.Event()
        self.move_event.clear()
        hang = float(self.Waittime.get())
        ang = self.Angle.get().split(',')
        print("Wait time = ", hang, ", ang = ", ang)

        ang = [s.strip() for s in ang]
        ang.sort()

        positions = []
        for phi in ang:
            pos = [phi]
            positions.append(pos)

        print(positions)

        # Check that everything is initialised
        initial = self.isInitialised()
        homed = self.isHomed()
        if not (homed*initial):
            return 1

        eventPhi = threading.Event()

        self.btnAutoMove["state"] = "disabled"

        self.automove_wait(hang, eventPhi, positions)

        return 0

    def next_position(self):
        self.move_event.set()

    def automove_wait(self, hang, eventPhi, positions, index = 0):
        #Move to position at index 0
        print("Moving to position", positions[index])
        self.automove_loop(eventPhi, positions[index])
        # If we are at the position we want to be
        if eventPhi.is_set():
            print("Switch on laser here!")
#            if self.laser != -1:
#                self.setLDStatus()
            if self.move_event.is_set():
                print("Switch off laser here!")
#                if self.laser != -1:
#                    self.setLDStatus()
                self.move_event.clear()
                #Update index to move to next position and clear all events
                index = index + 1
                eventPhi.clear()
                # Check if we are at the end of the sequence
                if index == len(positions):
                    self.btnAutoMove["state"] = "enabled"
                    print("Finished automove sequence!")
                    return 0
                else:
                    self.root.after(50, self.automove_wait, hang, eventPhi, positions, index)
            else:
                #Wait to collect data
                self.root.after(1000, self.automove_wait, hang, eventPhi, positions, index)
        # Not at position, so check again in 5 seconds
        else:
            # Wait until move has completed
            self.root.after(5000, self.automove_wait, hang, eventPhi, positions, index)

    def automove_loop(self, eventPhi, eventR, eventZ, position):
        if not eventPhi.is_set() and self.motor1.get_stopped(0):
            self.rotate(float(position[0]),eventPhi)
                  
        return 0

    def connectToLaser(self):
        dev = usb.core.find(idVendor=0x04d8, idProduct=0xfa73)
        if dev is None:
            print("Did not find laser device")
            raise ValueError('Device not found')
        
        for cfg in dev:
            for intf in cfg:
                if dev.is_kernel_driver_active(intf.bInterfaceNumber):
                    dev.detach_kernel_driver(intf.bInterfaceNumber)
        
        dev.reset()
        dev.set_configuration()
        
        self.laser = Laser(dev)
        self.laser.GetHardwareInfo()

        return 0

    def setLDCurrent(self):
        current = float(self.ldcurrentIn.get())
        self.laser.SetLDCurrent(current)
        return 0

    def setPulseWidth(self):
        pulse = float(self.pulsewidthIn.get())
        self.laser.SetPulseWidth(pulse/10.0)
        return 0

    def setLDStatus(self):
        onoff = 0
        if self.style.lookup("LD.TButton", "background") == 'red':
            self.style.configure("LD.TButton", foreground='white', background='green', text="Switch LD ON")
            self.btnldstatus.config(text='Switch LD ON')
            onoff = 0
        else:
            self.style.configure("LD.TButton", foreground='white', background='red', text="Switch LD OFF")
            self.btnldstatus.config(text='Switch LD OFF')
            onoff = 1

        self.laser.SetLDStatus(onoff)
        return 0

    def setTECStatus(self):
        onoff = 0
        if self.style.lookup("TEC.TButton", "background") == 'red':
            self.style.configure("TEC.TButton", foreground='white', background='green')
            self.btntecstatus.config(text='Switch TEC ON')
            onoff = 0
        else:
            self.style.configure("TEC.TButton", foreground='white', background='red')
            self.btntecstatus.config(text='Switch TEC OFF')
            onoff = 1

        self.laser.SetTECStatus(onoff)
        return 0

    def setExtTrig(self):
        onoff = 0
        if self.style.lookup("EXT.TButton", "background") == 'blue':
            self.style.configure("EXT.TButton", foreground='black', background='white')
            self.btnexttrig.config(text='Switch EXT ON')
            onoff = 0
            self.trig = -1
        else:
            self.style.configure("EXT.TButton", foreground='white', background='blue')
            self.btnexttrig.config(text='Switch EXT OFF')
            onoff = 1
            self.trig = 0

        self.laser.SetTriggerOnOff(0,0,onoff)
        return 0

    def setTriggerRate(self):
        rate = float(self.trigrateIn.get())
        if self.style.lookup("EXT.TButton", "background") == 'blue':
            print("Set to use external trigger, doing nothing")
            return 1

        if rate < 1 :
            self.laser.SetTriggerOnOff(0,0,0)
            self.trig = -1
            return 0
        elif (rate < 3) or (rate > 100000) :
            print("Cannot reduce trigger rate below 3kHz or above 100MHz")
            return 1
        elif rate < 100.1:
            self.laser.SetTriggerOnOff(0,1,0)
            self.laser.SetPG2Rate(rate)
            self.trig = 2
            return 0
        else:
            self.laser.SetTriggerOnOff(1,0,0)
            self.laser.SetPG1Rate(rate)
            self.trig = 1
            return 0

        return 0

    def getTriggerRate(self):
        if self.trig == 1:
            return float(self.laser.GetPG1Rate()/1000.0)
        elif self.trig == 2:
            return float(self.laser.GetPG2Rate()/1000.0)
        elif self.trig == -1:
            return "No trigger"
        else:
            return "EXT"

        return 0

    def __del__(self):
        self.stopall();
        self.root.quit()     # stops mainloop


root = Tk()
app = CDS_Control(root)
app.UpdatePos()

root.protocol("WM_DELETE_WINDOW", app.__del__)
root.mainloop()
