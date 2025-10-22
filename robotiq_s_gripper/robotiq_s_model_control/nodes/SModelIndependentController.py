#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""@package docstring
Enhanced command-line interface for sending commands to a ROS node controlling a S-Model gripper.

This controller extends the simple controller with individual finger control capabilities.
It provides both unified control (like the simple controller) and independent control 
of fingers A, B, C and the scissor axis. It supports:

- All simple controller functions (activate, reset, basic modes, unified control)
- Individual Control of Fingers (rICF): Independent control of fingers A, B, and C
- Individual Control of Scissor (rICS): Independent control of the scissor axis
- Per-finger position, speed, and force control
- Mode switching between unified and independent control

For advanced control modes and detailed documentation, refer to the Robotiq support website.
"""

import roslib; roslib.load_manifest('robotiq_s_model_control')
import rospy
from robotiq_s_model_articulated_msgs.msg import SModelRobotOutput 
from time import sleep

class SModelIndependentController:
    """Enhanced S-Model controller with individual finger control capabilities."""
    
    def __init__(self):
        """Initialize the controller with default parameters."""
        self.command = SModelRobotOutput()
        # Set default speeds for all fingers to maximum
        self.command.rSPA = 255
        self.command.rSPB = 255
        self.command.rSPC = 255
        self.command.rSPS = 255
        self.current_finger = 'A'  # Current finger for individual control ('A', 'B', 'C', 'S' for scissor)
        self.individual_mode = False  # Track if individual control is active
        self.scissor_mode = False    # Track if scissor control is active
        
    def genCommand(self, char, command):
        """Update the command according to the character entered by the user."""    
        
        # Basic activation and reset commands (unchanged from simple controller)
        if char == 'a':
            command = SModelRobotOutput()
            command.rACT = 1
            command.rGTO = 1
            command.rSPA = 255
            command.rSPB = 255
            command.rSPC = 255
            command.rSPS = 255
            command.rFRA = 150

        if char == 'r':
            command = SModelRobotOutput()
            command.rACT = 0

        # Basic unified control commands (unchanged from simple controller)
        if char == 'c':
            command.rPRA = 255

        if char == 'o':
            command.rPRA = 0

        # Grasping mode commands (unchanged from simple controller)
        if char == 'b':
            command.rMOD = 0
            
        if char == 'p':
            command.rMOD = 1
            
        if char == 'w':
            command.rMOD = 2
            
        if char == 's':
            command.rMOD = 3

        # Speed control for unified mode or current finger
        if char == 'f':
            if self.individual_mode:
                if self.current_finger == 'A':
                    command.rSPA = min(255, command.rSPA + 25)
                elif self.current_finger == 'B':
                    command.rSPB = min(255, command.rSPB + 25)
                elif self.current_finger == 'C':
                    command.rSPC = min(255, command.rSPC + 25)
                elif self.current_finger == 'S':
                    command.rSPS = min(255, command.rSPS + 25)
            else:
                command.rSPA = min(255, command.rSPA + 25)
                
        if char == 'l':
            if self.individual_mode:
                if self.current_finger == 'A':
                    command.rSPA = max(0, command.rSPA - 25)
                elif self.current_finger == 'B':
                    command.rSPB = max(0, command.rSPB - 25)
                elif self.current_finger == 'C':
                    command.rSPC = max(0, command.rSPC - 25)
                elif self.current_finger == 'S':
                    command.rSPS = max(0, command.rSPS - 25)
            else:
                command.rSPA = max(0, command.rSPA - 25)

        # Force control for unified mode or current finger
        if char == 'i':
            if self.individual_mode:
                if self.current_finger == 'A':
                    command.rFRA = min(255, command.rFRA + 25)
                elif self.current_finger == 'B':
                    command.rFRB = min(255, command.rFRB + 25)
                elif self.current_finger == 'C':
                    command.rFRC = min(255, command.rFRC + 25)
                elif self.current_finger == 'S':
                    command.rFRS = min(255, command.rFRS + 25)
            else:
                command.rFRA = min(255, command.rFRA + 25)
                
        if char == 'd':
            if self.individual_mode:
                if self.current_finger == 'A':
                    command.rFRA = max(0, command.rFRA - 25)
                elif self.current_finger == 'B':
                    command.rFRB = max(0, command.rFRB - 25)
                elif self.current_finger == 'C':
                    command.rFRC = max(0, command.rFRC - 25)
                elif self.current_finger == 'S':
                    command.rFRS = max(0, command.rFRS - 25)
            else:
                command.rFRA = max(0, command.rFRA - 25)

        # Individual control mode toggles
        if char == 'I':
            self.individual_mode = not self.individual_mode
            command.rICF = 1 if self.individual_mode else 0
            
        if char == 'S':
            self.scissor_mode = not self.scissor_mode
            command.rICS = 1 if self.scissor_mode else 0

        # Finger selection for individual control
        if char == '1':
            self.current_finger = 'A'
        if char == '2':
            self.current_finger = 'B'
        if char == '3':
            self.current_finger = 'C'
        if char == '4':
            self.current_finger = 'S'

        # Individual finger position control
        if char == '+':
            if self.individual_mode:
                if self.current_finger == 'A':
                    command.rPRA = min(255, command.rPRA + 10)
                elif self.current_finger == 'B':
                    command.rPRB = min(255, command.rPRB + 10)
                elif self.current_finger == 'C':
                    command.rPRC = min(255, command.rPRC + 10)
                elif self.current_finger == 'S':
                    command.rPRS = min(255, command.rPRS + 10)
            else:
                command.rPRA = min(255, command.rPRA + 10)
                
        if char == '-':
            if self.individual_mode:
                if self.current_finger == 'A':
                    command.rPRA = max(0, command.rPRA - 10)
                elif self.current_finger == 'B':
                    command.rPRB = max(0, command.rPRB - 10)
                elif self.current_finger == 'C':
                    command.rPRC = max(0, command.rPRC - 10)
                elif self.current_finger == 'S':
                    command.rPRS = max(0, command.rPRS - 10)
            else:
                command.rPRA = max(0, command.rPRA - 10)

        # Full open/close for current finger in individual mode
        if char == 'C':
            if self.individual_mode:
                if self.current_finger == 'A':
                    command.rPRA = 255
                elif self.current_finger == 'B':
                    command.rPRB = 255
                elif self.current_finger == 'C':
                    command.rPRC = 255
                elif self.current_finger == 'S':
                    command.rPRS = 255
                    
        if char == 'O':
            if self.individual_mode:
                if self.current_finger == 'A':
                    command.rPRA = 0
                elif self.current_finger == 'B':
                    command.rPRB = 0
                elif self.current_finger == 'C':
                    command.rPRC = 0
                elif self.current_finger == 'S':
                    command.rPRS = 0

        # Position input as integer (for unified mode or current finger)
        # Exclude finger selection numbers (1,2,3,4) from position commands
        try: 
            pos_value = int(char)
            # Skip if it's a finger selection command (1,2,3,4)
            if pos_value in [1, 2, 3, 4]:
                pass
            else:
                if pos_value > 255:
                    pos_value = 255
                if pos_value < 0:
                    pos_value = 0
                    
                if self.individual_mode:
                    if self.current_finger == 'A':
                        command.rPRA = pos_value
                    elif self.current_finger == 'B':
                        command.rPRB = pos_value
                    elif self.current_finger == 'C':
                        command.rPRC = pos_value
                    elif self.current_finger == 'S':
                        command.rPRS = pos_value
                else:
                    command.rPRA = pos_value
        except ValueError:
            pass                    

        return command

    def askForCommand(self, command):
        """Ask the user for a command to send to the gripper."""    

        currentCommand  = 'Enhanced S-Model Independent Controller\n'
        currentCommand += '=============================================\n'
        currentCommand += 'Control Mode: '
        if self.individual_mode:
            currentCommand += 'INDIVIDUAL FINGERS (rICF=1)'
        else:
            currentCommand += 'UNIFIED (rICF=0)'
        
        if self.scissor_mode:
            currentCommand += ' + SCISSOR (rICS=1)'
        currentCommand += '\n'
        
        if self.individual_mode:
            currentCommand += 'Active Finger: ' + self.current_finger + '\n'
        
        currentCommand += '-----\nCurrent command:\n'
        currentCommand += ' rACT = '  + str(command.rACT)
        currentCommand += ', rMOD = ' + str(command.rMOD)
        currentCommand += ', rGTO = ' + str(command.rGTO)
        currentCommand += ', rATR = ' + str(command.rATR)
        currentCommand += ', rGLV = ' + str(command.rGLV)
        currentCommand += ', rICF = ' + str(command.rICF)
        currentCommand += ', rICS = ' + str(command.rICS) + '\n'

        # Display finger positions, speeds, and forces
        currentCommand += 'Finger A: POS=' + str(command.rPRA).rjust(3)
        currentCommand += ', SPD=' + str(command.rSPA).rjust(3)
        currentCommand += ', FRC=' + str(command.rFRA).rjust(3) + '\n'
        
        currentCommand += 'Finger B: POS=' + str(command.rPRB).rjust(3)
        currentCommand += ', SPD=' + str(command.rSPB).rjust(3)
        currentCommand += ', FRC=' + str(command.rFRB).rjust(3) + '\n'
        
        currentCommand += 'Finger C: POS=' + str(command.rPRC).rjust(3)
        currentCommand += ', SPD=' + str(command.rSPC).rjust(3)
        currentCommand += ', FRC=' + str(command.rFRC).rjust(3) + '\n'
        
        currentCommand += 'Scissor:  POS=' + str(command.rPRS).rjust(3)
        currentCommand += ', SPD=' + str(command.rSPS).rjust(3)
        currentCommand += ', FRC=' + str(command.rFRS).rjust(3) + '\n'

        print(currentCommand)

        strAskForCommand  = '-----\nAvailable commands:\n\n'
        strAskForCommand += '=== BASIC CONTROL ===\n'
        strAskForCommand += 'r: Reset gripper\n'
        strAskForCommand += 'a: Activate gripper\n'
        strAskForCommand += 'c: Close (unified mode)\n'
        strAskForCommand += 'o: Open (unified mode)\n\n'
        
        strAskForCommand += '=== GRASPING MODES ===\n'
        strAskForCommand += 'b: Basic mode\n'
        strAskForCommand += 'p: Pinch mode\n'
        strAskForCommand += 'w: Wide mode\n'
        strAskForCommand += 's: Scissor mode\n\n'
        
        strAskForCommand += '=== INDIVIDUAL CONTROL ===\n'
        strAskForCommand += 'I: Toggle Individual Finger Control (rICF)\n'
        strAskForCommand += 'S: Toggle Scissor Control (rICS)\n'
        strAskForCommand += '1: Select Finger A\n'
        strAskForCommand += '2: Select Finger B\n'
        strAskForCommand += '3: Select Finger C\n'
        strAskForCommand += '4: Select Scissor axis\n\n'
        
        strAskForCommand += '=== POSITION CONTROL ===\n'
        strAskForCommand += '(0-255): Set position for unified mode or current finger\n'
        strAskForCommand += '+: Increase position by 10\n'
        strAskForCommand += '-: Decrease position by 10\n'
        strAskForCommand += 'C: Close current finger/axis fully\n'
        strAskForCommand += 'O: Open current finger/axis fully\n\n'
        
        strAskForCommand += '=== SPEED CONTROL ===\n'
        strAskForCommand += 'f: Increase speed\n'
        strAskForCommand += 'l: Decrease speed\n\n'
        
        strAskForCommand += '=== FORCE CONTROL ===\n'
        strAskForCommand += 'i: Increase force\n'
        strAskForCommand += 'd: Decrease force\n\n'
        
        strAskForCommand += '-->'

        return input(strAskForCommand)

    def publisher(self):
        """Main loop which requests new commands and publishes them on the SModelRobotOutput topic."""

        rospy.init_node('SModelIndependentController')

        topic_name = rospy.get_param('~topic', '/UR_1/SModelRobotOutput')
        # pub = rospy.Publisher(topic_name, SModelRobotOutput)
        pub = rospy.Publisher(topic_name, SModelRobotOutput, queue_size=1)

        while not rospy.is_shutdown():
            self.command = self.genCommand(self.askForCommand(self.command), self.command)            
            pub.publish(self.command)
            rospy.sleep(0.1)

def main():
    """Main entry point for the independent controller."""
    controller = SModelIndependentController()
    try:
        controller.publisher()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 