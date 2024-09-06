# CDS-Control-Code
These python scripts run the CDS control GUI.  There are two files, the CDS_Control class that creates the GUI and monitors the various threads controlling the motors and then the CDS_Motor class that communicates with the motor controller boards (Trinamic 3310 and 1180).

# CDS setup
Unplug USB, power on cabinet, reconnect usb, check encoders are green - vertical motion encoders will be blinking, the radial motion encoder will have a solid green light.

Run via:

  python CDS_Control_PlusStrain.py
  
This will start the GUI.
![image](https://github.com/user-attachments/assets/80191e65-fb6a-4fcc-bb5b-6657afcfab35)

To operate the system:

- Open Serial Ports to both motor controller boards
- Click the "Initialise Axis X" buttons to initialise the axes
   - The program will quit at this point if the serial ports are communicating with the wrong board
   - This happens occasionally when powering on
   - To fix, unplug and then re-plug the USB connection to the CDS electronics box
- Now click "Axis X Return to Home" for each axis
   - Start with axis 4 to ensure that the laser ball does not get caught on the car
   - If too much cable has been released then laserball may hit lower mPMT array - check that the Z axis rollers are close to their home positions before retracting the car
   - The Trinamics software can be used if needed to retract the Z axis before initialising the axis
- Now click the "Start Controller X Program" button to start the control thread to the motors

Known issues:
- If the Phi axis (Axis 3) encoder is incrementing without the axis moving, start the Controller 1 program as above then set the Axes 3 target position to be larger than the current encoder reading.
- Press the "Axis 3 Move to Position" button to start axis 3 rotating, then press "Stop Axis 3".
- The Axis 3 encoder should now have stopped incrementing by itself
- Press "Stop Controller 1 Program" button, then click "Axis 3 Return to Home" to reset the encoder counter.
- Start controller program 1 again to continue using the software

At this point the CDS system is ready to use.

# Usage
There are two operation modes of the software, one where you change each motor position individually, the other where you use the automated movement system.  

To use the automated movement please fill in the required positions in radius, angle and vertical drop using comma separated lists.  The code will loop through all combinations of these positions, moving the laser ball as requested.  At each phi angle, before the radial or vertical motion starts, the laser ball will move to the end of the arm, drop 60cm and then retract to the "zero" point just below the car.  It will then move to smaller radii, dropping the laser ball to the requested Z positions before retracting and moving to a shorter radius.

To use the individual motor movement, type a number into the "Axis X Target" box for the axis you want to move.  Then click "Axis X Move to Position".  

If necessary the axes can be stopped.  The "Stop Axis X" button will stop the single axis moving.  The "Stop All Axes" button will stop all axes.  

Note, if you need to return an axis to home for any point you must stop the relevant controller program first.  Also, be careful to ensure the laser ball does not get caught in the car if you retract axes 1 or 2 to home while the car is not at its home position.

# Emergency procedure
The control software can lose communication to the boards.  If the motors are moving consider cutting the power if there is any danger to the detector.  If the detector is safe then follow the procedure below:
- Kill the software
- Restart CDS_Control_PlusStrain.py
- Open serial ports
- Initialise all axes
- Press "Stop All Axes" to stop any motion

If necessary power to the motors can be cut using the cut-off switch on the electronics box.  When the motor power is restored the motors will continue moving in the same way as before, so only restart power when you are ready to send the stop command to the motor.

If necessary the Trinamics software can be started - the 3310 board can be communicated with via USB without power from the electronics box, allowing three axes to be stopped without motor motion.  The Trinamics software also has a red "Stop" button in the top left corner which will stop all connected motors.  Use this as soon as the power is switched back on to the electronics box.

If the software has been killed or closed in an unexpected state please remember to restart the CDS_Control program and intialise the motors again to ensure the limit switches and encoders are intialised correctly.


