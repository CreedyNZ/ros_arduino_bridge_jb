#!/usr/bin/env python

""" 
  Core code for Hexapod         
  Copyright (c)  2015 Andrew Creahan.  All rights reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software Foundation,
  Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
"""

import sys, time, os
import serial     
import p_servo
import config
import pygame
from espeak import espeak
from threading import Thread

import p_servo
delay = 0.1  # set rest time between command sends
checksum = 0
targetradius = 2

def play(file,*args):
  pygame.mixer.init()
  pygame.mixer.music.load(file)
  pygame.mixer.music.play()
  while pygame.mixer.music.get_busy() == True:
      continue
      
def servostart():
    p_servo.SetX(0)
    time.sleep(2) 
    p_servo.SetX(180)
    time.sleep(2)
    p_servo.SetX(90)
    time.sleep(2)
    p_servo.SetY(60)
    time.sleep(2) 
    p_servo.SetY(130)
    time.sleep(2)
    p_servo.SetY(90)
    time.sleep(2)
    p_servo.SetYD(70,20)
    time.sleep(5)
      
def startup():
    Thread(target=play, args=("/home/hexy/git/JimBob2/Python/Sounds/r2d2.ogg",1)).start()
    Thread(target=servostart).start()
    reset()

def reset():
  p_servo.ResetServo()
  
                            
                 

  
  
