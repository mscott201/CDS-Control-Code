#!/bin/python
# Parameters file for motors
import time
import control
import struct
import numpy as np
import usb.core
import usb.util
import sys

class Laser:

    def __init__(self, DEV):
        self.ID = 'COM6'# Initialise COM port to 0
        self.Temp60_40 = 0;
        self.None_LongShort = 1;
        self.PulseWidth_Fix = 2;
        self.AnalogTEC_ICTEC = 3;
        self.DA3SOA_DA3TECIMAX = 4;
        self.DA2_048V_DA3V = 5;
        self.TempRead10bit = 6;
        self.Invalid_Valid = 7;

        self.DEV = DEV

        self.HardInfo = -1
        
        print("Created Laser Object")

    # Method to send commands to motor controller board and get response
    def LASER_COMMAND(self, VALUE):

        TX = bytearray(66)
        for i in range(65):
            TX[i] = 255
        TX[65] = 0

        for i in range(len(VALUE)):
            TX[i] = VALUE[i]

        self.DEV.write(0x1,TX, 100)

        RX = bytearray(65)
        RX = self.DEV.read(0x81, 64, 100)

        return RX

    
    def GetFirmwareVersion(self):

        in_dat = bytearray(1)
        in_dat[0] = 1

        value = self.LASER_COMMAND(in_dat)
        print("x",int(str(value[0]),16),int(str(value[1]),16))

        return


    def GetHardwareInfo(self):

        in_dat = bytearray(3)
        in_dat[0] = 3
        in_dat[1] = 44
        in_dat[2] = 1

        value = self.LASER_COMMAND(in_dat)
    
        self.HardInfo = value[0]

        print("Got hardware")

        return


    def GetAlarms(self):
        indat = bytearray(1)
        indat[0]=9
        
        value = self.LASER_COMMAND(indat);
        alarms = [0,0,0,0]
        alarms[0] = ((values[2] & 0x10) == 0x10 );
        alarms[1] = ((values[2] & 0x20) == 0x20 );
        alarms[2] = ((values[2] & 0x40) == 0x40 );
        alarms[3] = ((values[2] & 0x80) == 0x80 );

        return alarms

    def GetPDCurrent(self):

        if self.HardInfo < 0:
            print("No Hardware Info")
            return 1

        indat = bytearray(1)
        indat[0]=9
        
        value = self.LASER_COMMAND(indat);

        num = 1.0
        if (self.HardInfo & (1 << self.Temp60_40)):
            num = 1.5

        curr = -1

        if (self.HardInfo & (1 << self.DA2_048V_DA3V)):
            curr = 10**(((float(value[0]) * 256.0 + float(value[1]) * 2.5) / 1023.0) - 0.5) / 0.2;

        else:
            curr = 10**(((float(value[0]) * 256.0 + float(value[1]) * 3.0) / 1023.0) - 0.5) / 0.2;

        return curr


    def GetLDTemp(self):

        if self.HardInfo < 0:
            print("No Hardware Info")
            return 1

        indat = bytearray(1)
        indat[0]=11
        
        value = self.LASER_COMMAND(indat);

        num = 1.0
        if (self.HardInfo & (1 << self.Temp60_40)):
            num = 1.5

        temp = -1

        if (self.HardInfo & (1 << self.DA2_048V_DA3V)):
            temp = (float(value[0]*256.0 + value[1])/1023.0) * 50.0 * num
        else:
            temp = (float(value[0]*256.0 + value[1])/1023.0) * 40.0 * num

        return temp

    def GetBDTemp(self):

        if self.HardInfo < 0:
            print("No Hardware Info")
            return 1

        indat = bytearray(1)
        indat[0]=12
        
        value = self.LASER_COMMAND(indat);

        num = 0.5
        num2 = 512.0
        if not (self.HardInfo & (1 << self.TempRead10bit)):
            num = 0.125
            num2 = 2048

        num3 = float(value[0]*256)

        temp = -1

        if (self.HardInfo & (1 << self.DA2_048V_DA3V)):
            temp = (float(value[0]*256.0 + value[1])/1023.0) * 50.0 * num
        else:
            temp = (float(value[0]*256.0 + value[1])/1023.0) * 40.0 * num

        return temp

    def GetLDStatus(self):

        if self.HardInfo < 0:
            print("No Hardware Info")
            return 1

        indat = bytearray(1)
        indat[0]=9
        
        value = self.LASER_COMMAND(indat);

        return (((value[15] & 4) == 4))

    def SetLDStatus(self, onoff):

        if self.HardInfo < 0:
            print("No Hardware Info")
            return 1

        indat = bytearray(2)
        indat[0]=6
        indat[1]=onoff
        
        value = self.LASER_COMMAND(indat);

        return 0

    def GetTECStatus(self):

        if self.HardInfo < 0:
            print("No Hardware Info")
            return 1

        indat = bytearray(1)
        indat[0]=9
        
        value = self.LASER_COMMAND(indat);

        return (((value[15] & 8) == 8))

    def SetTECStatus(self, onoff):

        if self.HardInfo < 0:
            print("No Hardware Info")
            return 1

        indat = bytearray(2)
        indat[0]=7
        indat[1]=onoff
        
        value = self.LASER_COMMAND(indat);

        return 0

    def SetBias(self, bias):

        if self.HardInfo < 0:
            print("No Hardware Info")
            return 1

        if (bias > 200.0):
            bias = 200.0

        indat = bytearray(3)
        indat[0]=19
        coe = 200.0
        if(self.HardInfo & (1 << self.DA2_048V_DA3V)):
            coe = 204.8
        indat[1] = ((int((bias * 4095.0 / coe)) >> 8) & 255)
        indat[2] = ((int((bias * 4095.0 / coe))     ) & 255)
       
        value = self.LASER_COMMAND(indat);

        return 0

    def GetBias(self):

        if self.HardInfo < 0:
            print("No Hardware Info")
            return 1

        indat = bytearray(3)
        indat[0]=19
        indat[1]=64
        indat[2]=0
       
        coe = 200.0
        if(self.HardInfo & (1 << self.DA2_048V_DA3V)):
            coe = 204.8

        values = self.LASER_COMMAND(indat);

        bias = (float)((values[0]) * 256 + values[1]) * coe / 4095.0;

        return bias

    def SetLDCurrent(self, current):

        if self.HardInfo < 0:
            print("No Hardware Info")
            return 1

        if (current > 200.0):
            current = 200.0

        indat = bytearray(3)
        indat[0]=20
        coe = 200.0
        if(self.HardInfo & (1 << self.DA2_048V_DA3V)):
            coe = 204.8

        indat[1] = ((int((current * 4095.0 / coe)) >> 8) & 255)
        indat[2] = ((int((current * 4095.0 / coe))     ) & 255)
       
        value = self.LASER_COMMAND(indat);

        return 0

    def GetLDCurrent(self):

        if self.HardInfo < 0:
            print("No Hardware Info")
            return 1

        indat = bytearray(3)
        indat[0]=20
        indat[1]=64
        indat[2]=0
       
        coe = 200.0
        if(self.HardInfo & (1 << self.DA2_048V_DA3V)):
            coe = 204.8

        values = self.LASER_COMMAND(indat);

        current = (float)((values[0]) * 256 + values[1]) * coe / 4095.0;

        return current

    def GetLongShort(self):

        indat = bytearray(2)
        indat[0]=34
        indat[1]=2
       
        values = self.LASER_COMMAND(indat);

        longshort = ((values[0]) == 1);

        return longshort

    def GetPulseWidth(self):

        indat = bytearray(1)
        indat[0]=9
       
        values = self.LASER_COMMAND(indat);

        pulsewidth = int((values[15] & 3)*256 + values[14]);

        return pulsewidth

    def SetPulseWidth(self, value):

        if value > 100:
            value = int(value / 10.0)

        value = int(value)

        indat = bytearray(4)
        indat[0]=8
        indat[1]=0 # Could be 0 or another num
        indat[2]= value >> 8
        indat[3]= value & 0xFF
 
        self.LASER_COMMAND(indat);

        print("Setting pulse width to", value)
        print("Pulse width = ", self.GetPulseWidth())

        return 0

    def SetTriggerOnOff(self, PG1=0, PG2=0, EXT=0):
        indat = bytearray(2)
        indat[0] = 14
        number = 0
        if PG1:
            number += 1
        if PG2:
            number += 2
        if EXT:
            number += 4
        indat[1] = number

        values = self.LASER_COMMAND(indat);
 
        return 0

 
    def SetPG1Rate(self, freq):

        freq = int(freq*1000)

        indat = bytearray(5)
        indat[0] = 15
        indat[1] = ((int(freq) >> 24) & 255)
        indat[2] = ((int(freq) >> 16) & 255)
        indat[3] = ((int(freq) >> 8) & 255)
        indat[4] = (int(freq) & 255)
    
        values = self.LASER_COMMAND(indat)

        return 0

    def GetPG1Rate(self):

        indat = bytearray(5)
        indat[0] = 15
        indat[1] = 128
        indat[2] = 0
        indat[3] = 0
        indat[4] = 0
    
        values = self.LASER_COMMAND(indat)

        if not ((values[0] == 255) & (values[1] == 255) & (values[2] == 255) & (values[3] == 255)):
            result = (int(values[0]) * 16777216) + (int(values[1]) * 65536) + (int(values[2]) * 256) + (int(values[3]));
                                       
        return result;

    def SetPG2Rate(self, freq):

        freq = int(freq*1000)
        print("PG2 Freq = ", freq)
        indat = bytearray(5)
        indat[0] = 16
        indat[1] = ((int(freq) >> 24) & 255)
        indat[2] = ((int(freq) >> 16) & 255)
        indat[3] = ((int(freq) >> 8) & 255)
        indat[4] = (int(freq) & 255)
    
        values = self.LASER_COMMAND(indat)

        return 0

    def GetPG2Rate(self):

        indat = bytearray(5)
        indat[0] = 16
        indat[1] = 128
        indat[2] = 0
        indat[3] = 0
        indat[4] = 0
    
        values = self.LASER_COMMAND(indat)

        if not ((values[0] == 255) & (values[1] == 255) & (values[2] == 255) & (values[3] == 255)):
            result = (int(values[0]) * 16777216) + (int(values[1]) * 65536) + (int(values[2]) * 256) + (int(values[3]));
                                       
        return result;


#dev = usb.core.find(idVendor=0x04d8, idProduct=0xfa73)
#if dev is None:
#        raise ValueError('Device not found')
#
##print(dev)
#
#for cfg in dev:
#    for intf in cfg:
#        if dev.is_kernel_driver_active(intf.bInterfaceNumber):
#            dev.detach_kernel_driver(intf.bInterfaceNumber)
#
#dev.reset()
#dev.set_configuration()
#
#laser = Laser(dev)
#
#laser.GetHardwareInfo()
#
#laser.SetTECStatus(0)
#
##laser.SetPG1Rate(1000000)
#print("PG1 rate =", laser.GetPG1Rate())
#laser.SetTriggerOnOff(1,0,0)
#
#print("Pulse width =", laser.GetPulseWidth())
#laser.SetPulseWidth(90)
#
#print("BD Temp = ", laser.GetBDTemp())
#print("LD Temp = ", laser.GetLDTemp())
#print("LD Current = ", laser.GetLDCurrent())
#
#print("LD Status = ", laser.GetLDStatus())
#print("TEC Status = ", laser.GetTECStatus())
#
#print("Bias = ", laser.GetBias())
#
#laser.SetTriggerOnOff(1,0,0)
#
#
#laser.SetLDCurrent(185.01)
#print("New LD Current = ", laser.GetLDCurrent())
#
#
#laser.SetLDStatus(0)
#
#
