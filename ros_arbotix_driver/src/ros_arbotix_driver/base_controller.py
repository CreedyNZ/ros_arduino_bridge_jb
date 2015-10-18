#!/usr/bin/env python

"""
    A base controller class for the Arduino microcontroller
    
    Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses
"""
import roslib; roslib.load_manifest('ros_arduino_python')
import rospy
import os

from math import sin, cos, pi
from geometry_msgs.msg import Transform, Twist, Pose
from ros_arduino_msgs.msg import PhoenixState

#Values for Phoenix Code Mcde setting
RESET  = 0
WALK = 10
ROTATE = 20
TRANSLATE = 30
SINLEG = 40
 
""" Class to receive Twist commands and publish Odometry data """
class BaseController:
    def __init__(self, ArbotixM, base_frame):
        self.arbotix = ArbotixM
        self.base_frame = base_frame
        self.rate = float(rospy.get_param("~base_controller_rate", 10))
        self.timeout = rospy.get_param("~base_controller_timeout", 1.0)
        self.stopped = False
        
        self.accel_limit = rospy.get_param('~accel_limit', 0.1)
                        
        now = rospy.Time.now()    
        self.then = now # time for determining dx/dy
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = now + self.t_delta

        # internal data        
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0                     # rotation in radians
        self.v_left = 0
        self.v_x = 0                    # cmd_vel setpoint
        self.v_y = 0
        self.t_x = 0                    # cmd_trans setpoint
        self.t_y = 0
        self.t_z = 0
        self.r_x = 0                    # cmd_trans rotate setpoint
        self.r_y = 0
        self.r_z = 0
        self.r_w = 0
        self.rotate = 0
        self.v_right = 0
        self.vel_calibrate = 127         #calibration factors for arbotix commands
        self.rot_calibrate = 50
        self.trans_calibrate = 50
        self.last_cmd_vel = now
        self.gait = 2         # Phoenix code variables
        self.balance = 0
        self.doubleT = 0
        self.stand = 0	

        # subscriptions
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)
        rospy.Subscriber("cmd_trans", Transform, self.cmdTransCallback)
        rospy.Subscriber("cmd_phstate", PhoenixState, self.cmdPhstateCallback)

        rospy.loginfo("Started base controller for a Hexapod base")
        
    
    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
                            
            dt = now - self.then
            self.then = now
            dt = dt.to_sec()
            
            # Set motor speeds in encoder ticks per PID loop
            if not self.stopped:
               if ((abs(self.t_x) + abs(self.t_y) + abs(self.t_z)) > 0):
                   self.arbotix.translate(self.t_x, self.t_y, self.t_z)
               elif ((abs(self.r_x) + abs(self.r_y) + abs(self.r_z)+ abs(self.r_w)) > 0):
                   self.arbotix.rotate(self.r_x, self.r_y, self.r_z, self.r_w)
               else
                   self.arbotix.travel(self.v_x, self.v_y, self.rotate)    
                   self.arbotix.state(self.balance, self.doubleT, self.stand)
                   self.arbotix.setgait(self.gait)           
                   print(self.v_x, self.v_y, self.rotate, self.balance, self.doubleT, self.stand, self.gait)
            self.t_next = now + self.t_delta
    
    def jbstart(self):    
        x = 0
        while x < 10:
           self.arbotix.state(0,0,0)
           time.sleep(0.1)
           x +=1        
            
    def stop(self):
        self.stopped = True
        self.arbotix.stop()
            
         
            
    def cmdVelCallback(self, req):
        # Handle velocity-based movement requests
        self.last_cmd_vel = rospy.Time.now()
        
        x = req.linear.x         # m/s
        y = req.linear.y         # m/s
        th = req.angular.z       # rad/s
        
            
        self.v_x = int(x * self.vel_calibrate)
        self.v_y = -int(y * self.vel_calibrate)
        self.rotate = -int(th * self.rot_calibrate)
        
    def cmdTransCallback(self, req):
        # Handle transform & rotation based movement requests
        self.last_cmd_vel = rospy.Time.now()
        
        tx = req.translation.x         
        ty = req.translation.y         
        tz = req.translation.z       # rad/s
        rx = req.rotation.x         
        ry = req.rotation.y         
        rz = req.rotation.z       
        rw = req.rotation.w       # rad/s
            
        self.t_x = int(tx * self.trans_calibrate)
        self.t_y = -int(ty * self.trans_calibrate)
        self.t_z = int(tz * self.trans_calibrate)
        self.r_x = int(rx * self.rot_calibrate)
        self.r_y = int(ry * self.rot_calibrate)
        self.r_z = int(rz * self.rot_calibrate)
        self.r_w = int(rw * self.rot_calibrate)   
    
    def cmdPhstateCallback(self, req):
        # Handle Phoenix code state commands
        
        self.gait = req.gait         # Phoenix commands
        self.balance = req.balance
        self.doubleT = req.doubleT
        self.stand = req.stand	
        

    

    
