#IMPORTS######################################################################
import threading
from CDS_Motor_PlusStrain import Motor
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
        self.trig = -1

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

        self.Port1 = StringVar(value = '/dev/ttyACM1')
        ttk.Entry(self.mainframe, textvariable=self.Port1).grid(column=2, row=0,columnspan=2, sticky=(W, E))
        ttk.Label(self.mainframe, text="Motor Controller 1 COM port:").grid(column=0, row=0,columnspan=2, sticky=W)

        self.Port2 = StringVar(value = '/dev/ttyACM0')
        ttk.Entry(self.mainframe, textvariable=self.Port2).grid(column=6, row=0, columnspan=2,sticky=(W, E))
        ttk.Label(self.mainframe, text="Motor Controller 2 COM port:").grid(column=4, row=0,columnspan=2, sticky=W)

        self.btnSerial1 = ttk.Button(self.mainframe, text="Open Controller 1 Serial Port",command=self.openSerial1)
        self.btnSerial1.grid(row=1,column=0,columnspan=2,pady=20, sticky=(N, W, E, S))
        self.btnSerial2 = ttk.Button(self.mainframe, text="Open Controller 2 Serial Port",command=self.openSerial2)
        self.btnSerial2.grid(row=1,column=4,columnspan=2,pady=20, sticky=(N, W, E, S))
        self.btnSerClo1 = ttk.Button(self.mainframe, text="Close Controller 1 Serial Port",command=self.closeSerial1)
        self.btnSerClo1.grid(row=1,column=2,columnspan=2,pady=20, sticky=(N, W, E, S))
        self.btnSerClo2 = ttk.Button(self.mainframe, text="Close Controller 2 Serial Port",command=self.closeSerial2)
        self.btnSerClo2.grid(row=1,column=6,columnspan=2,pady=20, sticky=(N, W, E, S))

#        self.mainframe.rowconfigure(2, minsize=30)

        btnMotor1 = ttk.Button(self.mainframe, text="Initialise Axis 1",command=self.InitialiseAxis1).grid(row=3,column=0,columnspan=2, sticky=(N, W, E, S))
        btnMotor2 = ttk.Button(self.mainframe, text="Initialise Axis 2",command=self.InitialiseAxis2).grid(row=3,column=2,columnspan=2, sticky=(N, W, E, S))
        btnMotor3 = ttk.Button(self.mainframe, text="Initialise Axis 3",command=self.InitialiseAxis3).grid(row=3,column=4,columnspan=2, sticky=(N, W, E, S))
        btnMotor4 = ttk.Button(self.mainframe, text="Initialise Axis 4",command=self.InitialiseAxis4).grid(row=3,column=6,columnspan=2, sticky=(N, W, E, S))

        self.btnHome1 = ttk.Button(self.mainframe, text="Axis 1 Return to Home",command=partial(self.home, 1))
        self.btnHome1.grid(row=4,column=0,columnspan=2, sticky=(N, W, E, S))
        self.btnHome2 = ttk.Button(self.mainframe, text="Axis 2 Return to Home",command=partial(self.home, 2))
        self.btnHome2.grid(row=4,column=2,columnspan=2, sticky=(N, W, E, S))
        self.btnHome3 = ttk.Button(self.mainframe, text="Axis 3 Return to Home",command=partial(self.home, 3))
        self.btnHome3.grid(row=4,column=4,columnspan=2, sticky=(N, W, E, S))
        self.btnHome4 = ttk.Button(self.mainframe, text="Axis 4 Return to Home",command=partial(self.home, 4))
        self.btnHome4.grid(row=4,column=6,columnspan=2, sticky=(N, W, E, S))

        self.PosEnc1 = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.PosEnc1).grid(column=1, row=5, sticky=(W, E))
        ttk.Label(self.mainframe, text="Axis 1 Encoder position:").grid(column=0, row=5, sticky=W)
 
        self.PosStep1 = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.PosStep1).grid(column=1, row=6, sticky=(W, E))
        ttk.Label(self.mainframe, text="Axis 1 step position:").grid(column=0, row=6, sticky=W)

        self.Target1 = StringVar(value='0')
        ttk.Entry(self.mainframe, textvariable=self.Target1).grid(column=1, row=7, sticky=(W, E))
        ttk.Label(self.mainframe, text="Axis 1 target:").grid(column=0, row=7, sticky=W)

        self.PosEnc2 = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.PosEnc2).grid(column=3, row=5, sticky=(W, E))
        ttk.Label(self.mainframe, text="Axis 2 Encoder position:").grid(column=2, row=5, sticky=W)
 
        self.PosStep2 = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.PosStep2).grid(column=3, row=6, sticky=(W, E))
        ttk.Label(self.mainframe, text="Axis 2 step position:").grid(column=2, row=6, sticky=W)

        self.Target2 = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.Target2).grid(column=3, row=7, sticky=(W, E))
        ttk.Label(self.mainframe, text="Axis 2 target:").grid(column=2, row=7, sticky=W)
 
        self.PosEnc3 = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.PosEnc3).grid(column=5, row=5, sticky=(W, E))
        ttk.Label(self.mainframe, text="Axis 3 Encoder position:").grid(column=4, row=5, sticky=W)
 
        self.PosStep3 = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.PosStep3).grid(column=5, row=6, sticky=(W, E))
        ttk.Label(self.mainframe, text="Axis 3 step position:").grid(column=4, row=6, sticky=W)

        self.Target3 = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.Target3).grid(column=5, row=7, sticky=(W, E))
        ttk.Label(self.mainframe, text="Axis 3 target:").grid(column=4, row=7, sticky=W)
 
        self.PosEnc4 = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.PosEnc4).grid(column=7, row=5, sticky=(W, E))
        ttk.Label(self.mainframe, text="Axis 4 Encoder position:").grid(column=6, row=5, sticky=W)
 
        self.PosStep4 = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.PosStep4).grid(column=7, row=6, sticky=(W, E))
        ttk.Label(self.mainframe, text="Axis 4 step position:").grid(column=6, row=6, sticky=W)

        self.Target4 = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.Target4).grid(column=7, row=7, sticky=(W, E))
        ttk.Label(self.mainframe, text="Axis 4 target:").grid(column=6, row=7, sticky=W)

#        self.mainframe.rowconfigure(8, minsize=30) 

        self.btnStart1 = ttk.Button(self.mainframe, text="Start Controller 1 Program",command=partial(self.start, 1))
        self.btnStart1.grid(row=9,column=0,columnspan=6, sticky=(N, W, E, S))
        self.btnStart2 = ttk.Button(self.mainframe, text="Start Controller 2 Program",command=partial(self.start, 2))
        self.btnStart2.grid(row=9,column=6,columnspan=2, sticky=(N, W, E, S))
        self.btnStpP1 = ttk.Button(self.mainframe, text="Stop Controller 1 Program",command=partial(self.stop, 1))
        self.btnStpP1.grid(row=10,column=0,columnspan=6, sticky=(N, W, E, S))
        self.btnStpP2 = ttk.Button(self.mainframe, text="Stop Controller 2 Program",command=partial(self.stop, 2))
        self.btnStpP2.grid(row=10,column=6,columnspan=2, sticky=(N, W, E, S))

#        self.mainframe.rowconfigure(11, minsize=30) 

        btnMove1 = ttk.Button(self.mainframe, text="Axis 1 Move to Position",command=partial(self.singlemove, 1)).grid(row=12,column=0,columnspan=2, sticky=(N, W, E, S))
        btnMove2 = ttk.Button(self.mainframe, text="Axis 2 Move to Position",command=partial(self.singlemove, 2)).grid(row=12,column=2,columnspan=2, sticky=(N, W, E, S))
        btnMove3 = ttk.Button(self.mainframe, text="Axis 3 Move to Position",command=partial(self.singlemove, 3)).grid(row=12,column=4,columnspan=2, sticky=(N, W, E, S))
        btnMove4 = ttk.Button(self.mainframe, text="Axis 4 Move to Position",command=partial(self.singlemove, 4)).grid(row=12,column=6,columnspan=2, sticky=(N, W, E, S))

        btnStop1  = ttk.Button(self.mainframe, text="Stop Axis 1",command=partial(self.stopaxis, 1)).grid(row=13,column=0,columnspan=2, sticky=(N, W, E, S))
        btnStop2  = ttk.Button(self.mainframe, text="Stop Axis 2",command=partial(self.stopaxis, 2)).grid(row=13,column=2,columnspan=2, sticky=(N, W, E, S))
        btnStop3  = ttk.Button(self.mainframe, text="Stop Axis 3",command=partial(self.stopaxis, 3)).grid(row=13,column=4,columnspan=2, sticky=(N, W, E, S))
        btnStop4  = ttk.Button(self.mainframe, text="Stop Axis 4",command=partial(self.stopaxis, 4)).grid(row=13,column=6,columnspan=2, sticky=(N, W, E, S))

        self.btnStopAll  = ttk.Button(self.mainframe, text="Stop All Axes",command=self.stopall)
        self.btnStopAll.grid(row=14,column=0,columnspan=8, sticky=(N, W, E, S))
        button = ttk.Button(master=self.mainframe, text="Quit", command=self.__del__).grid(row=15,column=0,columnspan=8, sticky=(N, W, E, S))

        self.fig1 = Figure(figsize=(9, 4), dpi=self.root.winfo_fpixels('1i'))
        ax1 = self.fig1.add_subplot(2,4,1)
        ax2 = self.fig1.add_subplot(2,4,2)
        ax3 = self.fig1.add_subplot(2,4,3)
        ax4 = self.fig1.add_subplot(2,4,4)
        ax5 = self.fig1.add_subplot(2,4,5)
        ax6 = self.fig1.add_subplot(2,4,6)
        ax7 = self.fig1.add_subplot(2,4,7)
        ax8 = self.fig1.add_subplot(2,4,8)
        ax = [ax1, ax2, ax3, ax4, ax5, ax6, ax7, ax8]
 
        for index in range(8):
            if(index == 0):
                ax[index].set_ylim(-10, 200)
                ax[index].set_ylabel("Upper Feed Position / cm")
            elif(index == 1):
                ax[index].set_ylim(-10, 200)
                ax[index].set_ylabel("Lower Feed Position / cm")
            elif(index == 2):
                ax[index].set_ylim(-180, 180)
                ax[index].set_ylabel("Arm Angle / degrees")
            elif(index == 3):
                ax[index].set_ylim(-10, 200)
                ax[index].set_ylabel("Radius / cm")
            else:
                ax[index].set_ylim(-10,1000)
                ax[index].set_ylabel("1/Motor Load")

            ax[index].set_xlim(0, 100)
            ax[index].set_xlabel("Time / s")
            ax[index].grid()

        self.SetPlots(ax1, ax2, ax3, ax4, ax5, ax6, ax7, ax8)
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

        #self.mainframe.columnconfigure(9, minsize=20, weight = 1) 

        self.Axis1LimitLeft = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.Axis1LimitLeft, width=15).grid(column=9, row=0, sticky=(W))
        self.Axis1LimitLeftLabel = ttk.Label(self.mainframe, text="Axis 1 Left Limit:")
        self.Axis1LimitLeftLabel.grid(column=8, row=0, sticky=E)

        self.Axis1LimitRight = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.Axis1LimitRight, width=15).grid(column=9, row=1, sticky=(W))
        self.Axis1LimitRightLabel = ttk.Label(self.mainframe, text="Axis 1 Right Limit:")
        self.Axis1LimitRightLabel.grid(column=8, row=1, sticky=E)

        self.Axis2LimitLeft = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.Axis2LimitLeft, width=15).grid(column=9, row=2, sticky=(W))
        self.Axis2LimitLeftLabel = ttk.Label(self.mainframe, text="Axis 2 Left Limit:")
        self.Axis2LimitLeftLabel.grid(column=8, row=2, sticky=E)

        self.Axis2LimitRight = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.Axis2LimitRight, width=15).grid(column=9, row=3, sticky=(W))
        self.Axis2LimitRightLabel = ttk.Label(self.mainframe, text="Axis 2 Right Limit:")
        self.Axis2LimitRightLabel.grid(column=8, row=3, sticky=E)

        self.Axis3LimitLeft = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.Axis3LimitLeft, width=15).grid(column=9, row=4, sticky=(W))
        self.Axis3LimitLeftLabel = ttk.Label(self.mainframe, text="Axis 3 Left Limit:")
        self.Axis3LimitLeftLabel.grid(column=8, row=4, sticky=E)

        self.Axis3LimitRight = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.Axis3LimitRight, width=15).grid(column=9, row=5, sticky=(W))
        self.Axis3LimitRightLabel = ttk.Label(self.mainframe, text="Axis 3 Right Limit:")
        self.Axis3LimitRightLabel.grid(column=8, row=5, sticky=E)

        self.Axis4LimitLeft = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.Axis4LimitLeft, width=15).grid(column=9, row=6, sticky=(W))
        self.Axis4LimitLeftLabel = ttk.Label(self.mainframe, text="Axis 4 Left Limit:")
        self.Axis4LimitLeftLabel.grid(column=8, row=6, sticky=E)

        self.Axis4LimitRight = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.Axis4LimitRight, width=15).grid(column=9, row=7, sticky=(W))
        self.Axis4LimitRightLabel = ttk.Label(self.mainframe, text="Axis 4 Right Limit:")
        self.Axis4LimitRightLabel.grid(column=8, row=7, sticky=E)

#        self.mainframe.rowconfigure(8, minsize=20) 

        self.Waittime = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.Waittime, width=15).grid(column=9, row=9, sticky=(W))
        self.WaittimeLabel = ttk.Label(self.mainframe, text="Wait time:")
        self.WaittimeLabel.grid(column=8, row=9, sticky=E)

        self.Radius = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.Radius, width=15).grid(column=9, row=10, sticky=(W))
        self.RadiusLabel = ttk.Label(self.mainframe, text="Radius:")
        self.RadiusLabel.grid(column=8, row=10, sticky=E)

        self.Angle = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.Angle, width=15).grid(column=9, row=11, sticky=(W))
        self.AngleLabel = ttk.Label(self.mainframe, text="Angle:")
        self.AngleLabel.grid(column=8, row=11, sticky=E)

        self.ZPosition = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.ZPosition, width=15).grid(column=9, row=12, sticky=(W))
        self.ZPositionLabel = ttk.Label(self.mainframe, text="Z Position:")
        self.ZPositionLabel.grid(column=8, row=12, sticky=E)

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
        self.btnldcurrent.grid(row=3,column=11,columnspan=2, sticky=(N, W, E, S))

        self.trigrate = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.trigrate, width=15).grid(column=12, row=5, sticky=(W))
        self.trigRateLabel = ttk.Label(self.mainframe, text="Trigger Rate (kHz)")
        self.trigRateLabel.grid(column=11, row=5, sticky=E)
        self.btntrigrate = ttk.Button(self.mainframe, text="Set Trig Rate",command=self.setTriggerRate)
        self.btntrigrate.grid(row=6,column=11,columnspan=2, sticky=(N, W, E, S))

        self.pulsewidth = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.pulsewidth, width=15).grid(column=12, row=8, sticky=(W))
        self.pulseWidthLabel = ttk.Label(self.mainframe, text="Pulse Width (ps)")
        self.pulseWidthLabel.grid(column=11, row=8, sticky=E)
        self.btnpulsewidth = ttk.Button(self.mainframe, text="Set Pulse Width",command=self.setPulseWidth)
        self.btnpulsewidth.grid(row=9,column=11,columnspan=2, sticky=(N, W, E, S))

        self.laserPDCurrent = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.laserPDCurrent, width=15).grid(column=12, row=11, sticky=(W))
        self.laserPDCurrentLabel = ttk.Label(self.mainframe, text="PD Current (mA)")
        self.laserPDCurrentLabel.grid(column=11, row=11, sticky=E)

        self.btnldstatus = ttk.Button(self.mainframe, text="Switch LD ON", command=self.setLDStatus, style="LD.TButton")
        self.btnldstatus.grid(row=13,column=11,columnspan=2, sticky=(N, W, E, S))

        self.btntecstatus = ttk.Button(self.mainframe, text="Switch TEC ON",command=self.setTECStatus, style="TEC.TButton")
        self.btntecstatus.grid(row=14,column=11,columnspan=2, sticky=(N, W, E, S))

        self.btnexttrig = ttk.Button(self.mainframe, text="Switch EXT ON",command=self.setExtTrig, style="EXT.TButton")
        self.btnexttrig.grid(row=15,column=11,columnspan=2, sticky=(N, W, E, S))


        self.mainframe.rowconfigure(16, weight=1)
#        self.mainframe.columnconfigure(11, weight=1)

        y = [35.6, 23.0, 14.2, 7.0, 64.7, 50.0, 4.3, 1.1, 0.6, 2.5, 67.4]#, 70.0, 80.0, 90.0, 100.0]
        x = [46.4, 36.1, 27.25, 19.1, 75.4, 58.9, 15.7, 8.7, 3.25, 13.25, 78.4]
        x.sort()
        y.sort()
        self.rspline = interpolate.splrep(x, y, s=0)
        xnew = np.arange(0,100,0.1)
        ynew = interpolate.splev(xnew,self.rspline, der=0)
#        plt.plot(x,y)
#        plt.plot(xnew,ynew)
#        plt.show()

        self.radius = -1
        self.angle = 0
        self.zpos = -10

        # Initial zero point of laser ball from home to just below car.  Needs re-calculating at each point.
        self.zero_radius_height_offset = 0.0 # was 61.0, measured now as 58.5cm, set to zero
        # Distance from centre of roller to feet is 82.8cm, so 24.3cm different
        self.zero_height = self.zero_radius_height_offset 

        return 
 
    def anim(self, i):
        # function to update figures
        lines = []

        if(self.hasMotor1):
            lines = self.motor1.animate()
        if(self.hasMotor2):
            line4 = self.motor2.animate()
            lines.insert(3, line4[0])
            lines.append(line4[1])

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

    def openSerial2(self):
        inPort = self.Port2.get()
        self.ser2 = 0
        self.ser2 = serial.Serial(
                    port=inPort,
                    baudrate=115200,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    timeout=2
        )
        print("Serial Port",inPort, "has been opened")
        self.hasMotor2 = 1;
        self.btnSerial2["state"] = "disabled"
        self.btnSerClo2["state"] = "normal"
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

    def closeSerial2(self):
        if(self.hasMotor2):
            self.stop(2)
        if(self.ser2.isOpen()):
            self.ser2.close()
        print("Serial Port 2 has been closed")
        self.hasMotor2 = 0;
        self.btnSerClo2["state"] = "disabled"
        self.btnSerial2["state"] = "normal"
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

    def InitialiseAxis2(self):
        axis = 1
        port = self.Port1.get()
        if(not self.hasMotor1):
            print("Cannot initialise axis without opening serial port")
            return 1

        self.motor1.INIT_AXIS(self.ser1, port, axis, 'Z')
        self.hasMotor1Axis2 = 1
        return 0

    def InitialiseAxis3(self):
        axis = 2
        port = self.Port1.get()
        if(not self.hasMotor1):
            print("Cannot initialise axis without opening serial port")
            return 1

        self.motor1.INIT_AXIS(self.ser1, port, axis, 'PHI')
        self.hasMotor1Axis3 = 1
        return 0

    def InitialiseAxis4(self):
        axis = 0
        port = self.Port2.get()
        if(not self.hasMotor2):
            print("Cannot initialise axis without opening serial port")
            return 1

        self.motor2.INIT_AXIS(self.ser2, port, axis, 'R')
        self.hasMotor2Axis1 = 1
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

        if(self.hasMotor1Axis2):
            self.PosEnc2.set(str(self.motor1.get_encoder_position(1)))
            self.PosStep2.set(str(self.motor1.get_step_position(1)))

            left = self.motor1.get_left_limit_status(1)
            right = self.motor1.get_right_limit_status(1)
            if(left):
                self.Axis2LimitLeftLabel.configure(style="error.TLabel")
            else:
                self.Axis2LimitLeftLabel.configure(style="TLabel")
            if(right):
                self.Axis2LimitRightLabel.configure(style="error.TLabel")
            else:
                self.Axis2LimitRightLabel.configure(style="TLabel")

            self.Axis2LimitLeft.set(str(left))
            self.Axis2LimitRight.set(str(right))

        if(self.hasMotor1Axis3):
            self.PosEnc3.set(str(self.motor1.get_encoder_position(2)))
            self.PosStep3.set(str(self.motor1.get_step_position(2)))

            left = self.motor1.get_left_limit_status(2)
            right = self.motor1.get_right_limit_status(2)
            if(left):
                self.Axis3LimitLeftLabel.configure(style="error.TLabel")
            else:
                self.Axis3LimitLeftLabel.configure(style="TLabel")
            if(right):
                self.Axis3LimitRightLabel.configure(style="error.TLabel")
            else:
                self.Axis3LimitRightLabel.configure(style="TLabel")

            self.Axis3LimitLeft.set(str(left))
            self.Axis3LimitRight.set(str(right))

        if(self.hasMotor2Axis1):
            self.PosEnc4.set(str(self.motor2.get_encoder_position(0)))
            self.PosStep4.set(str(self.motor2.get_step_position(0)))
 
            left = self.motor2.get_left_limit_status(0)
            right = self.motor2.get_right_limit_status(0)
            if(left):
                self.Axis4LimitLeftLabel.configure(style="error.TLabel")
            else:
                self.Axis4LimitLeftLabel.configure(style="TLabel")
            if(right):
                self.Axis4LimitRightLabel.configure(style="error.TLabel")
            else:
                self.Axis4LimitRightLabel.configure(style="TLabel")

            self.Axis4LimitLeft.set(str(left))
            self.Axis4LimitRight.set(str(right))

        if(self.laser != -1):
            self.laserPDCurrent.set(str(self.laser.GetPDCurrent()))
            self.pulsewidth.set(str(self.laser.GetPulseWidth()))
            self.trigrate.set(str(self.laser.GetTrigRate()))
            self.ldcurrent.set(str(self.laser.GetLDCurrent()))

        self.root.after(500, app.UpdatePos)

        return 0

    def SetPlots(self, ax1, ax2, ax3, ax4, ax5, ax6, ax7, ax8):
        axes = [ax1, ax2, ax3]
        strains = [ax5, ax6, ax7]
        self.motor1.SetPlot(axes, strains)
        axes2 = [ax4]
        strains2 = [ax8]
        self.motor2.SetPlot(axes2, strains2)
        return 0

    def start(self, motor_nb):
        #this is the main controller thread
        if(self.hasMotor1 and self.ControlThread1 == 0 and motor_nb == 1):
            self.motor1.set_active()
            self.ControlThread1 = threading.Thread(target=self.motor1.CONTROLLER_TICK, name='Motor1')
            self.ControlThread1.start()
            self.btnStpP1["state"] = "normal"
            self.btnStart1["state"] = "disabled"
           
        elif(self.hasMotor2 and self.ControlThread2 == 0 and motor_nb == 2):
            self.motor2.set_active()
            self.ControlThread2 = threading.Thread(target=self.motor2.CONTROLLER_TICK, name='Motor2')
            self.ControlThread2.start()
            self.btnStpP2["state"] = "normal"
            self.btnStart2["state"] = "disabled"

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

        elif(self.hasMotor2 and motor_nb == 2):
            self.motor2.disable_axis(0)
            time.sleep(0.25)
            self.motor2.STOP()
            if(self.ControlThread2):
                self.motor2.set_inactive()
                self.ControlThread2 = 0
                self.btnStpP2["state"] = "disabled"
                self.btnStart2["state"] = "normal"
        else:
            print("Trying to stop a controller that has not been initialised")
            return 1
        print("Have stopped controller", motor_nb)
        return 0        

    def stopaxis(self, axis_nb):
        if(self.hasMotor1 and axis_nb < 4):
            self.motor1.disable_axis(axis_nb-1)
        elif(self.hasMotor2 and axis_nb == 4):
            self.motor2.disable_axis(0)
        else:
            print("Trying to stop a axis that has not been initialised")
            return 1
        print("Have stopped axis", axis_nb)
        return 0

    def stopall(self):
        self.stop(1)
        self.stop(2)
        return 0

    def singlemove(self, motor_nb):
        if(motor_nb == 1):
            position = float(self.Target1.get())
            return self.move(motor_nb, position)
        if(motor_nb == 2):
            position = float(self.Target2.get())
            return self.move(motor_nb, position)
        if(motor_nb == 3):
            position = float(self.Target3.get())
            return self.move(motor_nb, position)
        if(motor_nb == 4):
            position = float(self.Target4.get())
            return self.move(motor_nb, position)


    def move(self, motor_nb, position):
        if(motor_nb < 4):
            if(not self.hasMotor1):
                print("Please initialise Motor controller 1 to move axis", motor_nb)
                return 1
            elif(not self.ControlThread1):
                print("Please start controller program 1 to move axis", motor_nb)
                return 1

        if(motor_nb ==4):
            if(not self.hasMotor2):
                print("Please initialise Motor controller 2 to move axis", motor_nb)
                return 1
            elif(not self.ControlThread2):
                print("Please start controller program 2 to move axis", motor_nb)
                return 1

        if(motor_nb < 4):
            self.motor1.enable_axis(motor_nb-1)
        elif(motor_nb == 4):
            self.motor2.enable_axis(0)

        if(motor_nb == 1):
            print("Moving axis 1 to position: ", self.motor1.set_target_position(position,0))
        elif(motor_nb == 2):
            print("Moving axis 2 to position: ", self.motor1.set_target_position(position,1))
        elif(motor_nb == 3):
            print("Moving axis 3 to position: ", self.motor1.set_target_position(position,2))
        elif(motor_nb == 4):
            print("Moving axis 4 to position: ", self.motor2.set_target_position(position,0))

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

        elif (motor_nb == 2):
            if(self.hasMotor1Axis2):
                self.motor1.set_active()
                event = threading.Event()
                ControlThread = threading.Thread(target=self.motor1.SendToHome, args=(event,1,), name='Motor1Home')
                ControlThread.start()
                self.disable_frame()
                self.event_wait_loop(event,2,0)
                self.zpos = -10
            else:
                print("Cannot send motor home without initialising axis")
                return 1

        elif (motor_nb == 3):
            if(self.hasMotor1Axis3):
                self.motor1.set_active()
                event = threading.Event()
                ControlThread = threading.Thread(target=self.motor1.SendToHome, args=(event,2,), name='Motor1Home')
                ControlThread.start()
                self.disable_frame()
                self.event_wait_loop(event,3,0)
            else:
                print("Cannot send motor home without initialising axis")
                return 1
               
        elif (motor_nb == 4):
            if(self.hasMotor2Axis1):
                self.motor2.set_active()
                event = threading.Event()
                ControlThread = threading.Thread(target=self.motor2.SendToHome, args=(event,0,), name='Motor2Home')
                ControlThread.start()
                self.disable_frame()
                self.event_wait_loop(event,4,0)
            else:
                print("Cannot send motor 2 home without initialising axis")
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
        initial = (self.hasMotor1Axis1 * self.hasMotor1Axis2 * self.hasMotor1Axis3 * self.hasMotor2Axis1)
        if not initial:
            print("Please initialise all motor axes before using the Automove function")
        return initial

    def rotate(self, phi, event):
        if self.motor1.get_stopped(2):
            print("Moving to angle", phi)
            self.angle = phi
            self.move(3, phi)
            time.sleep(0.5)

        if self.motor1.isSetPointReached(2):
            event.set()
            print("Moved to angle", phi)
        if not event.is_set():
            self.root.after(500, self.rotate, phi, event)

        return 0

    def moveZ(self, z, event):

        z_backlash = (float(self.radius)/111.6)*1.6 + 1.2
        
        realPos = self.zero_height + z

        if z > 1:
            realPos += z_backlash

        if self.motor1.get_stopped(0) and self.motor1.get_stopped(1):
            print("Moving to z position", z)
            # Move each Z axis to half the distance
            # Removes need to monitor individual axes and limit switches.
            self.move(1, realPos/2.0)
            self.move(2, realPos/2.0)       
            time.sleep(0.5)

        if self.motor1.isSetPointReached(0) and self.motor1.isSetPointReached(1):
            event.set()
            self.zpos = realPos
            print("Moved to Z position", z)
        if not event.is_set():
            self.root.after(500, self.moveZ, z, event)

        return 0


    def moveRadius(self, r, eventR, eventZ):
        print("Moving to radius", r)
        realR = r
        if r > self.radius:
            realR += 1.0

        self.radius = r
        realR += 8.3 # Add distance from limit switch to wheel on car

        if realR > 120:
            print("Moving off end of arm")
            exit(0)

        #Calculate "height" of ball below car
        # Use spline for most points, but linear extrapolation either end
        # Zero is at zero radius and ball just below car.
#        if r < 3.25:
#            height = 0.001846153846*r
#        elif r > 59.0:
#            height = 0.892*r -2.53
#        else:
#            height = interpolate.splev(r, self.rspline, der=0)

        # New calc, final day at IC
        height = 55.5 + 0.686*r + 1.89e-3*r*r


        # Get zero height for current position
        curr_zheight = self.zero_height
        # Calculate Z position needed to place laser ball just below car
        self.zero_height = self.zero_radius_height_offset + height
        eventR.clear()
        eventZ.clear()
        self.moveRadius_wait(r, realR, curr_zheight, eventR, eventZ)

        return 0

    def moveRadius_wait(self, r, realR, curr_zheight, eventR, eventZ):
        # If zposition of laserball is too short for intended R position
        if self.zpos < self.zero_height and self.motor1.get_stopped(0) and self.motor1.get_stopped(1):
            # Re-calculated zero_height, therefore only move 2mm below that
            self.moveZ(0.2, eventZ)

        # Or if laserball Z is too long and not moving
        elif self.zpos > (self.zero_height + 200) and self.motor1.get_stopped(0) and self.motor1.get_stopped(1):
            #  If current Z is too long, retract to current z-height
            self.moveZ(curr_zheight + 0.05 - self.zero_height, eventZ)

        elif self.zpos > 0:
            eventZ.set()

        if eventZ.is_set():
            if not eventR.is_set() and self.motor2.get_stopped(0):
                self.move(4, realR)
                time.sleep(0.5)
            if self.motor2.isSetPointReached(0):
                eventR.set()
                eventZ.clear()
                print("Moved to radius", r)
            if not eventR.is_set():
                self.root.after(500, self.moveRadius_wait,r, realR, curr_zheight, eventR, eventZ)
        else:
            self.root.after(500, self.moveRadius_wait,r, realR, curr_zheight, eventR, eventZ)

        return 0

    def automove(self):
        self.move_event = threading.Event()
        self.move_event.clear()
        hang = float(self.Waittime.get())
        rads = self.Radius.get().split(',')
        ang = self.Angle.get().split(',')
        zpos = self.ZPosition.get().split(',')
        print("Wait time = ", hang, ", rads = ", rads, ", ang = ", ang, ", zpos = ", zpos)

        rads = [s.strip() for s in rads]
        ang = [s.strip() for s in ang]
        zpos = [s.strip() for s in zpos]

        rads.sort(reverse=True)
        ang.sort()
        zpos.sort()

        positions = []
        for phi in ang:
            positions.append([phi,0,0])
            positions.append([phi,55,0])
            positions.append([phi,110,0])
            positions.append([phi,110,60])
            positions.append([phi,110,0])
            for r in rads:
                for i in range(len(zpos)):
                    pos = [phi, r, zpos[i]]
                    positions.append(pos)
                    if i == (len(zpos)-1):
                        print("HELLO")
                        pos2 = [phi, r, 0]
                        positions.append(pos2)


        print(positions)

        # Check that everything is initialised
        initial = self.isInitialised()
        homed = self.isHomed()
        if not (homed*initial):
            return 1

        eventPhi = threading.Event()
        eventR = threading.Event()
        eventZ = threading.Event()

        self.btnAutoMove["state"] = "disabled"

        self.automove_wait(hang, eventPhi, eventR, eventZ, positions)

        return 0

    def automove_wait(self, hang, eventPhi, eventR, eventZ, positions, index = 0):
        #Move to position at index 0
        print("Moving to position", positions[index])
        self.automove_loop(eventPhi, eventR, eventZ, positions[index])
        # If we are at the position we want to be
        if eventPhi.is_set() and eventR.is_set() and eventZ.is_set():
            print("Switch on laser here!")
            time.sleep(hang)
            print("Switch off laser here!")
            #Update index to move to next position and clear all events
            index = index + 1
            eventPhi.clear()
            eventR.clear()
            eventZ.clear()
            # Check if we are at the end of the sequence
            if index == len(positions):
                self.btnAutoMove["state"] = "enabled"
                print("Finished automove sequence!")
                return 0
            else:
                self.root.after(50, self.automove_wait, hang, eventPhi, eventR, eventZ, positions, index)
        # Not at position, so check again in 5 seconds
        else:
            # Wait until move has completed
            self.root.after(5000, self.automove_wait, hang, eventPhi, eventR, eventZ, positions, index)

    def next_position(self):
        self.move_event.set()

    def automove_wait(self, hang, eventPhi, eventR, eventZ, positions, index = 0):
        #Move to position at index 0
        print("Moving to position", positions[index])
        self.automove_loop(eventPhi, eventR, eventZ, positions[index])
        # If we are at the position we want to be
        if eventPhi.is_set() and eventR.is_set() and eventZ.is_set():
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
                eventR.clear()
                eventZ.clear()
                # Check if we are at the end of the sequence
                if index == len(positions):
                    self.btnAutoMove["state"] = "enabled"
                    print("Finished automove sequence!")
                    return 0
                else:
                    self.root.after(50, self.automove_wait, hang, eventPhi, eventR, eventZ, positions, index)
            else:
                #Wait to collect data
                self.root.after(1000, self.automove_wait, hang, eventPhi, eventR, eventZ, positions, index)
        # Not at position, so check again in 5 seconds
        else:
            # Wait until move has completed
            self.root.after(5000, self.automove_wait, hang, eventPhi, eventR, eventZ, positions, index)

    def automove_loop(self, eventPhi, eventR, eventZ, position):
        if not eventPhi.is_set() and self.motor1.get_stopped(2):
            self.rotate(float(position[0]),eventPhi)
             
        if eventPhi.is_set():
            # For Radial motion we also move in Z for part of it, so make sure Z motors are not moving too
            if not eventR.is_set() and (self.motor2.get_stopped(0) and self.motor1.get_stopped(0) and self.motor1.get_stopped(1)):
                self.moveRadius(float(position[1]), eventR, eventZ)
                
        if eventPhi.is_set() and eventR.is_set():
            if not eventZ.is_set() and self.motor1.get_stopped(0) and self.motor1.get_stopped(1):
                self.moveZ(float(position[2]), eventZ)
       
        return 0

    def connectToLaser(self):
        dev = usb.core.find(idVendor=0x04d8, idProduct=0xfa73)
        if dev is None:
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
        current = float(self.ldcurrent.get())
        self.laser.SetLDCurrent(current)
        return 0

    def setPulseWidth(self):
        pulse = float(self.pulsewidth.get())
        self.laser.SetPulseWidth(pulse)
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
        rate = float(self.trigrate.get())
        if self.style.lookup("EXT.TButton", "background") == 'blue':
            print("Set to use external trigger, doing nothing")
            return 1
        if (rate < 2) or (rate > 10000) :
            print("Cannot reduce trigger rate below 2kHz or above 10MHz")
            return 1
        elif rate < 50:
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
            return self.laser.GetPG1Rate()
        elif self.trig == 2:
            return self.laser.GetPG2Rate()
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
