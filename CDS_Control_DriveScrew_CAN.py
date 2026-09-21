#IMPORTS######################################################################
import threading
from CDS_Motor_DriveScrew_CAN import Motor
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
import time
from scipy import interpolate
import usb.core
import usb.util
import sys
import os
import can

#%% Initiate motor drive
# Axes in this class are labelled from 1 to 4.
# Axes in CDS_Motor are labelled from 0 to 2, since each card has max 3 axes and they start at 0.
class CDS_Control():

    def __init__(self, root):

        self.move_event = -1

        self.current_state = []

        self.style = ttk.Style()
        self.style.theme_use('alt')
        self.style.configure("error.TLabel", foreground='white', background='red')

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

        self.btnSerial1 = ttk.Button(self.mainframe, text="Open Controller 1 CAN Port",command=self.openSerial1)
        self.btnSerial1.grid(row=0,column=0,columnspan=4,pady=20, sticky=(N, W, E, S))
        self.btnSerClo1 = ttk.Button(self.mainframe, text="Close Controller 1 CAN Port",command=self.closeSerial1)
        self.btnSerClo1.grid(row=1,column=0,columnspan=4,pady=20, sticky=(N, W, E, S))

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

        self.btnStart1 = ttk.Button(self.mainframe, text="Start Controller",command=partial(self.start, 1))
        self.btnStart1.grid(row=9,column=0,columnspan=4, sticky=(N, W, E, S))
        self.btnStpP1 = ttk.Button(self.mainframe, text="Stop Controller",command=partial(self.stop, 1))
        self.btnStpP1.grid(row=10,column=0,columnspan=4, sticky=(N, W, E, S))

        btnMove1 = ttk.Button(self.mainframe, text="Axis 1 Move to Position",command=partial(self.singlemove, 1)).grid(row=12,column=0,columnspan=2, sticky=(N, W, E, S))
        btnStop1  = ttk.Button(self.mainframe, text="Stop Axis 1",command=partial(self.stopaxis, 1)).grid(row=13,column=0,columnspan=2, sticky=(N, W, E, S))

        btnMotor2 = ttk.Button(self.mainframe, text="Initialise Axis 2",command=self.InitialiseAxis2).grid(row=3,column=2,columnspan=2, sticky=(N, W, E, S))

        self.btnHome2 = ttk.Button(self.mainframe, text="Axis 2 Return to Home",command=partial(self.home, 2))
        self.btnHome2.grid(row=4,column=2,columnspan=2, sticky=(N, W, E, S))

        self.PosEnc2 = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.PosEnc2).grid(column=3, row=5, sticky=(W, E))
        ttk.Label(self.mainframe, text="Axis 2 Encoder position:").grid(column=2, row=5, sticky=W)
 
        self.PosStep2 = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.PosStep2).grid(column=3, row=6, sticky=(W, E))
        ttk.Label(self.mainframe, text="Axis 2 step position:").grid(column=2, row=6, sticky=W)

        self.Target2 = StringVar(value='0')
        ttk.Entry(self.mainframe, textvariable=self.Target2).grid(column=3, row=7, sticky=(W, E))
        ttk.Label(self.mainframe, text="Axis 2 target:").grid(column=2, row=7, sticky=W)

        btnMove2 = ttk.Button(self.mainframe, text="Axis 2 Move to Position",command=partial(self.singlemove, 2)).grid(row=12,column=2,columnspan=2, sticky=(N, W, E, S))
        btnStop2  = ttk.Button(self.mainframe, text="Stop Axis 2",command=partial(self.stopaxis, 2)).grid(row=13,column=2,columnspan=2, sticky=(N, W, E, S))

        self.btnStopAll  = ttk.Button(self.mainframe, text="Stop All Axes",command=self.stopall)
        self.btnStopAll.grid(row=14,column=0,columnspan=7, sticky=(N, W, E, S))
        button = ttk.Button(master=self.mainframe, text="Quit", command=self.__del__).grid(row=15,column=0,columnspan=7, sticky=(N, W, E, S))

        self.fig1 = Figure(figsize=(9, 4), dpi=self.root.winfo_fpixels('1i'))
        ax1 = self.fig1.add_subplot(2,2,1)
        axa = self.fig1.add_subplot(2,2,3)
        ax2 = self.fig1.add_subplot(2,2,2)
        axb = self.fig1.add_subplot(2,2,4)
        ax = [ax1, ax2, axa, axb]
 
        self.radius = -999
        self.angle = -999

        for index in range(4):
            if(index == 0):
                ax[index].set_ylim(-10, 250)
                ax[index].set_ylabel("Screw carriage Position / cm")
            elif(index == 1):
                ax[index].set_ylim(-10, 370)
                ax[index].set_ylabel("Rotation Position / degree")

            else:
                ax[index].set_ylim(-10,1000)
                ax[index].set_ylabel("1/Motor Load")

            ax[index].set_xlim(0, 100)
            ax[index].set_xlabel("Time / s")
            ax[index].grid()

        self.SetPlots(ax1, ax2, axa, axb)
        self.fig1.tight_layout()
        self.canvas1 = FigureCanvasTkAgg(self.fig1, master=self.root)  # A tk.DrawingArea.
        self.canvas1.draw()
        toolbar1 = NavigationToolbar2Tk(self.canvas1, self.root, pack_toolbar=False)
        toolbar1.update()
        self.canvas1.mpl_connect("key_press_event", lambda event: print(f"you pressed {event.key}"))
        self.canvas1.mpl_connect("key_press_event", key_press_handler)
        self.canvas1.get_tk_widget().grid(row=16,column=0,columnspan=4, sticky=(N, S, E, W))

        self.ani = animation.FuncAnimation(self.fig1, self.anim, interval=50, blit=False, save_count=10)

        self.Axis1LimitLeft = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.Axis1LimitLeft).grid(column=6, row=0, sticky=(W))
        self.Axis1LimitLeftLabel = ttk.Label(self.mainframe, text="Axis 1 Left Limit:")
        self.Axis1LimitLeftLabel.grid(column=5, row=0, sticky=E)

        self.Axis1LimitRight = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.Axis1LimitRight).grid(column=6, row=1, sticky=(W))
        self.Axis1LimitRightLabel = ttk.Label(self.mainframe, text="Axis 1 Right Limit:")
        self.Axis1LimitRightLabel.grid(column=5, row=1, sticky=E)

        self.Axis2LimitLeft = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.Axis2LimitLeft).grid(column=6, row=4, sticky=(W))
        self.Axis2LimitLeftLabel = ttk.Label(self.mainframe, text="Axis 2 Left Limit:")
        self.Axis2LimitLeftLabel.grid(column=5, row=4, sticky=E)

        self.Axis2LimitRight = StringVar()
        ttk.Entry(self.mainframe, textvariable=self.Axis2LimitRight).grid(column=6, row=5, sticky=(W))
        self.Axis2LimitRightLabel = ttk.Label(self.mainframe, text="Axis 2 Right Limit:")
        self.Axis2LimitRightLabel.grid(column=5, row=5, sticky=E)

        self.Waittime = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.Waittime).grid(column=6, row=7, sticky=(W))
        self.WaittimeLabel = ttk.Label(self.mainframe, text="Wait time:")
        self.WaittimeLabel.grid(column=5, row=7, sticky=E)

        self.Radius = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.Radius).grid(column=6, row=8, sticky=(W))
        self.RadiusLabel = ttk.Label(self.mainframe, text="Radius:")
        self.RadiusLabel.grid(column=5, row=8, sticky=E)

        self.Angle = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.Angle).grid(column=6, row=9, sticky=(W))
        self.AngleLabel = ttk.Label(self.mainframe, text="Angle:")
        self.AngleLabel.grid(column=5, row=9, sticky=E)

        self.Repeats = StringVar(value = '0')
        ttk.Entry(self.mainframe, textvariable=self.Repeats).grid(column=6, row=10, sticky=(W))
        self.WaittimeLabel = ttk.Label(self.mainframe, text="No. of reps:")
        self.WaittimeLabel.grid(column=5, row=10, sticky=E)

        self.btnAutoMove  = ttk.Button(self.mainframe, text="Start Auto Movement",command=self.automove)
        self.btnAutoMove.grid(row=11,column=5,columnspan=2, sticky=(N, W, E, S))
        self.btnMovePosition  = ttk.Button(self.mainframe, text="Next Position",command=self.next_position)
        self.btnMovePosition.grid(row=12,column=5,columnspan=2, sticky=(N, W, E, S))
        self.btnStopAuto  = ttk.Button(self.mainframe, text="Stop Auto Movement",command=self.stop_automove)
        self.btnStopAuto.grid(row=13,column=5, columnspan=2, sticky=(N, W, E, S))

        for child in self.mainframe.winfo_children(): 
            child.grid_configure(padx=5, pady=5)

        self.mainframe.columnconfigure(3, minsize=200, weight = 0) 

        self.mainframe.rowconfigure(16, weight=1)

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
        os.system('sudo ifconfig can0 down')
        os.system('sudo ip link set can0 type can bitrate 1000000')
        os.system("sudo ifconfig can0 txqueuelen 100000")
        os.system('sudo ifconfig can0 up')

        self.ser1 = can.ThreadSafeBus(channel = 'can0', interface = 'socketcan')

        print("CAN Port can0 has been opened")
        self.hasMotor1 = 1;
        self.btnSerial1["state"] = "disabled"
        self.btnSerClo1["state"] = "normal"
        return 0

    def closeSerial1(self):
        if(self.hasMotor1):
            self.stop(1)

        self.ser1.shutdown()
        print("CAN Port can0 has been closed")
        self.hasMotor1 = 0;
        self.btnSerClo1["state"] = "disabled"
        self.btnSerial1["state"] = "normal"

        os.system('sudo ifconfig can0 down')

        return 0

    def InitialiseAxis1(self):
        axis = 0
        if(not self.hasMotor1):
            print("Cannot initialise axis without opening CAN bus")
            return 1

        self.motor1.INIT_AXIS(self.ser1, axis, 'SCREW')
        self.hasMotor1Axis1 = 1
        return 0

    def InitialiseAxis2(self):
        axis = 1
        if(not self.hasMotor1):
            print("Cannot initialise axis without opening CAN bus")
            return 1

        self.motor1.INIT_AXIS(self.ser1, axis, 'PHI')
        self.hasMotor1Axis2 = 1
        return 0

    def InitialiseAxis3(self):
        axis = 2
        if(not self.hasMotor1):
            print("Cannot initialise axis without opening CAN bus")
            return 1

#        self.motor1.INIT_AXIS(self.ser1, axis, 'PHI')
#        self.hasMotor1Axis3 = 1
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

        self.root.after(500, app.UpdatePos)

        return 0

    def SetPlots(self, ax1, ax2, axa, axb):
        axes = [ax1, ax2]
        strains = [axa, axb]
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
            self.motor1.set_inactive()
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
        if hasattr(self, 'start_auto'):
            self.start_auto.clear()
        self.stop(1)
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

    def move(self, motor_nb, position):
        if(not self.hasMotor1):
            print("Please initialise Motor controller 1 to move axis", motor_nb)
            return 1
        elif(not self.ControlThread1):
            print("Please start controller program 1 to move axis", motor_nb)
            return 1

        self.motor1.enable_axis(motor_nb-1)

        if(motor_nb == 1):
            print("Moving axis 1 to position: ", self.motor1.set_target_position(position,0))
        elif(motor_nb == 2):
            print("Moving axis 2 to position: ", self.motor1.set_target_position(position,1))
        elif(motor_nb == 3):
            print("Moving axis 3 to position: ", self.motor1.set_target_position(position,2))
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
                if not self.ControlThread1==0:
                    print("Stop the Control Thread 1 before sending axis to home")
                    return 1
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

        else:
            print("Trying to home non-existent motor...")
            return 1;

        print(self.homed)

        return 0

    def disable_frame(self):
        self.current_state = []
        for child in self.mainframe.winfo_children():
            self.current_state.append(child.cget("state"))
            child.configure(state="disabled")
        self.btnStopAll.configure(state="enable")

    def enable_frame(self):
        count = 0
        for child in self.mainframe.winfo_children():
            child.configure(state=self.current_state[count])
            count = count + 1

    def isHomed(self):
        hom = 1
        for home in self.homed:
            hom *= home
            break
        if not hom:
            print("Please send each axis to home before trying to move them.")
        return hom 

    def isInitialised(self):
        initial = (self.hasMotor1Axis1) # * self.hasMotor1Axis2 * self.hasMotor1Axis3 * self.hasMotor2Axis1)
        if not initial:
            print("Please initialise all motor axes before using the Automove function")
        return initial

    def automove(self):
        self.move_event = threading.Event()
        self.move_event.clear()
        self.start_auto = threading.Event()
        self.start_auto.set()
        self.radius = -9999
        self.angle = -9999


        hang = float(self.Waittime.get())
        rads = self.Radius.get().split(',')
        angs = self.Angle.get().split(',')
        reps = int(self.Repeats.get())
        print("Wait time = ", hang, ", rads = ", rads, ", angs = ", angs, "repeats = ", reps)

        rads = [s.strip() for s in rads]
        rads.sort(reverse=True)

        angs = [s.strip() for s in angs]

        positions = []
        counter = 0;
        while counter < reps:
            for a in angs:
                for r in rads:
                    if int(r) > 210:
                        r = 210
                    pos = [a, r]
                    positions.append(pos)
            counter = counter + 1

        print(positions)

        # Check that everything is initialised
        initial = self.isInitialised()
        homed = self.isHomed()
        if not (homed*initial):
            return 1

        eventR = threading.Event()
        eventR.clear()

        eventA = threading.Event()
        eventA.clear()

        moving = threading.Event()
        moving.clear()

        self.btnAutoMove["state"] = "disabled"

        self.automove_wait(hang, eventR, eventA, moving, positions)

        return 0

    def stop_automove(self):
        self.start_auto.clear()
        return 0

    def automove_wait(self, hang, eventR, eventA, moving, positions, index = 0):
        #Move to position at index 0
        print("Moving to position", positions[index])
        r = int(positions[index][1])
        angle = float(positions[index][0])

        #Check if the move was stopped, and if so exit move.
        if not self.start_auto.is_set():
            self.btnAutoMove["state"] = "enabled"
            self.stopall()
            print("Stopped automove sequence!")
            return 0;

        if not eventR.is_set():
            if self.motor1.get_stopped(0) and abs(self.radius - r) > 0.5 and not moving.is_set():
                print("Moving to radius", r)
                self.move(1, r)
                moving.set()
                self.root.after(5000, self.automove_wait, hang, eventR, eventA, moving, positions, index)
                return 0

            if self.motor1.isSetPointReached(0):
                print("Moved to radius", r)
                eventR.set()
                moving.clear()
                self.radius = r
                self.root.after(100, self.automove_wait, hang, eventR, eventA, moving, positions, index)
                return 0
            # Not at position, so check again in 5 seconds
            else:
                # Wait until move has completed
                self.root.after(5000, self.automove_wait, hang, eventR, eventA, moving, positions, index)
                return 0

        if not eventA.is_set():
            if self.motor1.get_stopped(1) and abs(self.angle - angle) > 0.1 and not moving.is_set():
                print("Moving to angle", angle)
                self.move(2, angle)
                moving.set()
                self.root.after(5000, self.automove_wait, hang, eventR, eventA, moving, positions, index)
                return 0

            if self.motor1.isSetPointReached(1):
                print("Moved to angle", angle)
                eventA.set()
                moving.clear()
                self.angle = angle
                self.root.after(100, self.automove_wait, hang, eventR, eventA, moving, positions, index)
                return 0
            # Not at position, so check again in 5 seconds
            else:
                # Wait until move has completed
                self.root.after(5000, self.automove_wait, hang, eventR, eventA, moving, positions, index)
                return 0

        else:
            print("Switch on laser here!")
            if hang > 0:
                time.sleep(hang)
                print("Switch off laser here!")
                #Update index to move to next position and clear all events
                index = index + 1
                eventR.clear()
                eventA.clear()
                moving.clear()
                if index == len(positions):
                    self.btnAutoMove["state"] = "enabled"
                    print("Finished automove sequence!")
                    return 0
                else:
                    self.root.after(100, self.automove_wait, hang, eventR, eventA, moving, positions, index)
                    return 0

            elif self.move_event.is_set():
                self.move_event.clear()
                print("Switch off laser here!")
                #Update index to move to next position and clear all events
                index = index + 1
                eventR.clear()
                eventA.clear()
                moving.clear()
                if index == len(positions):
                    self.btnAutoMove["state"] = "enabled"
                    print("Finished automove sequence!")
                    return 0
                else:
                    self.root.after(100, self.automove_wait, hang, eventR, eventA, moving, positions, index)
                    return 0

            else:
                #Wait to collect data
                self.root.after(1000, self.automove_wait, hang, eventR, eventA, moving, positions, index)
                return 0


    def next_position(self):
        self.move_event.set()

    def automove_loop(self, eventR, r):
        if not self.start_auto.is_set():
            print("Stopping automove")
            self.stopall()
            return 0


        if not eventR.is_set():
            self.root.after(500, self.automove_loop, eventR, r)

        return 0


    def __del__(self):
        self.stopall();
        self.root.quit()     # stops mainloop


root = Tk()
app = CDS_Control(root)
app.UpdatePos()

root.protocol("WM_DELETE_WINDOW", app.__del__)
root.mainloop()
