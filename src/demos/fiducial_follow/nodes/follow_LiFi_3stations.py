#!/usr/bin/python

"""
Copyright (c) 2017, Ubiquity Robotics
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of fiducial_follow nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

"""
Fiducial Follow Demo.  Receives trasforms to fiducials and generates
movement commands for the robot to follow the fiducial of interest.
"""

import rospy
from geometry_msgs.msg import TransformStamped, Twist
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
import tf2_ros
from math import pi, sqrt, atan2
import traceback
import math
import time
import subprocess
import os
import socket

import errno

from time import sleep

from  mac_list  import *

#subprocess.call('sudo -S python ~/catkin_ws/src/demos/fiducial_follow/nodes/switchbot.py cb:87:fa:95:c6:0b Bot Press', shell=True)
#os.system("sudo python switchbot.py cb:87:fa:95:c6:0b Bot Press")
#print("-----------------Check Point------------")

#=================================================================================================
# take the server name and port name
#host = 'local host'
#host = socket.gethostname()
#port = 5000
  
# create a socket at client side
# using TCP / IP protocol
#s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
  
# connect it to server and port
# number on local computer.
#s.connect(('192.168.0.139', port))
#s.connect((host, port))
  
# receive message string from
# server, at a time 1024 
#msg = s.recv(1024)
#==================================================================================================

##time.sleep(5000)
import pygame
mp3_path ='/home/ubuntu/catkin_ws/src/demos/fiducial_follow/nodes/'
play_dest_received_first_time = 1
def play_dest_received          (station):
    global play_dest_reached_first_time
    global play_dest_received_first_time
    print("++++++++++++++++++++++++++++++++++")
    print("try to play play_dest_received()", station)
    print("++++++++++++++++++++++++++++++++++")
    if play_dest_received_first_time == 0:
        return
    play_dest_received_first_time  = 0
    play_dest_reached_first_time   = 1
    if station =='B':
        sound_file = "B.wav"
    elif station =='C':
        sound_file = "C.wav"
        sound_file = "C.mp3"
    elif station =='A':
        sound_file = "A.wav"
    elif station =='D':
        sound_file = "D.wav"
    elif station =='E':
        sound_file = "E.wav"
    else:
        return 
    print("Playing ", mp3_path+ sound_file)
    pygame.mixer.init()
    pygame.mixer.music.load(mp3_path+ sound_file)
    pygame.mixer.music.set_volume(1.0)
    pygame.mixer.music.play()

    while pygame.mixer.music.get_busy() == True:
            pass
    return
  

play_dest_reached_first_time  = 1
def play_dest_reached(station):
    global play_dest_reached_first_time
    global play_dest_received_first_time
    print("%%%%%%%%%%%%%%%%%%%%%%%%%++++++++++++++++++++++++++++++++++")
    print("try to play play_dest_reached()", station)
    print("++++++++++++++++++++++++++++++++++%%%%%%%%%%%%%%%%%%%%%%%%%\n")
    if play_dest_reached_first_time == 0:
        return
    
    play_dest_reached_first_time   = 0
    play_dest_received_first_time  = 1
    if station =='B':
        sound_file = "BSB.wav"
    elif station =='C':
        sound_file = "BSC.wav"
    elif station =='A':
        sound_file = "BSA.wav"
    elif station =='D':
        sound_file = "BSD.wav"
    elif station =='E':
        sound_file = "BSE.wav"
    elif station == "STOP":
        sound_file = "last_station.wav"
    else:
        return 
    pygame.mixer.init()
    pygame.mixer.music.load(mp3_path + sound_file)
    pygame.mixer.music.set_volume(1.0)
    pygame.mixer.music.play()

    while pygame.mixer.music.get_busy() == True:
            pass
    return


def degrees(r):
    return 180.0 * r / math.pi

class Follow:
    """
    Constructor for our class
    """
    def __init__(self):
       rospy.init_node('follow')

       # Set up a transform listener so we can lookup transforms in the past
       self.tfBuffer = tf2_ros.Buffer(rospy.Time(30))
       self.lr = tf2_ros.TransformListener(self.tfBuffer)

       # Setup a transform broadcaster so that we can publish transforms
       # This allows to visualize the 3D position of the fiducial easily in rviz
       self.br = tf2_ros.TransformBroadcaster()

       # A publisher for robot motion commands
       self.cmdPub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

       # Flag to avoid sending repeated zero speeds
       self.suppressCmd = False

       # The name of the coordinate frame of the fiducial we are interested in (original was  "fid49")
       self.target_fiducial = rospy.get_param("~target_fiducial", "fid101")

       # Minimum distance we want the robot to be from the fiducial
       self.min_dist = rospy.get_param("~min_dist", 0.6)

       # Maximum distance a fiducial can be away for us to attempt to follow
       self.max_dist = rospy.get_param("~max_dist", 2.5)

       # Proportion of angular error to use as angular velocity
       self.angular_rate = rospy.get_param("~angular_rate", 2.0)

       # Maximum angular speed (radians/second)
       self.max_angular_rate = rospy.get_param("~max_angular_rate", 1.2)

       # Angular velocity when a fiducial is not in view
       self.lost_angular_rate = rospy.get_param("~lost_angular_rate", 0.6)

       # Proportion of linear error to use as linear velocity
       self.linear_rate = rospy.get_param("~linear_rate", 1.2)

       # Maximum linear speed (meters/second)
       self.max_linear_rate = rospy.get_param("~max_linear_rate", 1.5)

       # Linear speed decay (meters/second)
       self.linear_decay = rospy.get_param("~linear_decay", 0.9)

       # How many loop iterations to keep linear velocity after fiducial
       # disappears
       self.hysteresis_count = rospy.get_param("~hysteresis_count", 20)

       # How many loop iterations to keep rotating after fiducial disappears
       self.max_lost_count = rospy.get_param("~max_lost_count", 400)

       # Subscribe to incoming transforms
       rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.newTf)
       self.fid_x = self.min_dist
       self.fid_y = 0
       self.got_fid = False


    """
    Called when a FiducialTransformArray is received
    """
    def newTf(self, msg):
        imageTime = msg.header.stamp
        self.linSpeed = 0

        #print imageTime, rospy.Time.now()
        #print "*****"
        found = False

        # For every fiducial found by the dectector, publish a transform
        for m in msg.transforms:
            id = m.fiducial_id
            self.fidID = id
            trans = m.transform.translation
            rot = m.transform.rotation
##            print "Fid %d %lf %lf %lf %lf %lf %lf %lf\n" % \
##                                 (id, trans.x, trans.y, trans.z,
##                                  rot.x, rot.y, rot.z, rot.w)
            t = TransformStamped()
            t.child_frame_id = "fid%d" % id
            t.header.frame_id = msg.header.frame_id
            t.header.stamp = imageTime
            t.transform.translation.x = trans.x
            t.transform.translation.y = trans.y
            t.transform.translation.z = trans.z
            t.transform.rotation.x = rot.x
            t.transform.rotation.y = rot.y
            t.transform.rotation.z = rot.z
            t.transform.rotation.w = rot.w
            self.br.sendTransform(t)

            if t.child_frame_id == self.target_fiducial:
                # We found the fiducial we are looking for
                found = True

                # Add the transform of the fiducial to our buffer
                self.tfBuffer.set_transform(t, "follow")

        if not found:
            return # Exit this function now, we don't see the fiducial
        try:
            # Get the fiducial position relative to the robot center, instead of the camera
            tf = self.tfBuffer.lookup_transform("base_link", self.target_fiducial, imageTime)
            ct = tf.transform.translation
            cr = tf.transform.rotation
            print "T_fidBase %lf %lf %lf %lf %lf %lf %lf\n" % \
                             (ct.x, ct.y, ct.z, cr.x, cr.y, cr.z, cr.w)

            # Set the state varibles to the position of the fiducial
            self.fid_x = ct.x   ######### Tmp: This part was ct.x to ct.z Husam modified it 
            self.fid_y = ct.y   ######### Tmp: This part was ct.y to ct.x Husam modified it
            self.got_fid = True
        except:
            traceback.print_exc()
            print "Could not get tf for %s" % self.target_fiducial


    """
    Main loop
    """
    def run(self):
        # setup for looping at 20hz
        rate = rospy.Rate(20)

        # Setup the variables that we will use later
        linSpeed = 0.0
        angSpeed = 0.0
        times_since_last_fid = 0

        destFlag = False
        destination = 1
        global MAC_list_status
        global MAC_list_name
        global MAC_list_role
        global play_dest_reached_first_time
        m = "\x30\x31\x39\x32\x2e\x31\x36\x38\x2e\x30\x2e\x31\x37\x32"
        m = "No"
        #counter = 0

        # While our node is running
        #===================================================================
        try:
            
            s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setblocking(False)
            #s.settimeout(1)
            s.bind(('',12345))
        except:
            print ("Opeening Socket Error ");
            #sys.exit(0);
        #socket = s
        m = "\x30\x31\x39\x32\x2e\x31\x36\x38\x2e\x30\x2e\x31\x35\x37"
        destination = "192.168.0.157"
        destFlag = False
        #print(MAC_list_role)
        for mac, role in MAC_list_role.items():
            print(mac, role)
            if role == 'Destination':
                print("*******************************")
                print( mac, MAC_list_name[mac])
                play_dest_received(MAC_list_name[mac])
                print("*******************************")
                #sleep(1)
        #return
        last_station = ''                
        while not rospy.is_shutdown():
            try:
                #play_dest_received()
                #msg = s.recv(4096)
                print("Before Receive")
                #m=s.recv(1024)

                print("After Receive")
                #msg1=m.decode('utf-8')
                print('\n++++++++++++++++++++++++++++++++++++++\n' )
                #print('Received:' + msg1)

                #print("MAC_list_status", MAC_list_status)
                
                for mac, state in get_mac_list_().items():
                    if destFlag == True:
                        break
                    role =  MAC_list_role[mac]
                    name = MAC_list_name[mac]
                    print(mac, name, state, role)
                    #sleep(1)
                    dest = 0
                   
                    if (state == 'UP' ) & (role == 'Destination'):
                        print( mac, MAC_list_role[mac])
                        if last_station != MAC_list_name[mac]:
                             play_dest_reached(MAC_list_name[mac])
                             play_dest_reached_first_time  = 1
                             play_dest_reached("STOP")
                             last_station = MAC_list_name[mac]
                             play_dest_reached_first_time  = 1
                        stop_keep_looping(True)
                        destFlag = True
                        #play_dest_reached_first_time  = 1
                        
                        
                    elif (state == 'UP' ) & (role == 'passby'):
                        if last_station != MAC_list_name[mac]:
                             play_dest_reached(MAC_list_name[mac])
                             last_station = MAC_list_name[mac]
                             play_dest_reached_first_time  = 1
                        destFlag = False
                    else:
                        destFlag = False
                all_down = 1
                for mac, state in get_mac_list_().items():
                     if (state == 'UP' ):
                         all_down = 0
                if all_down ==1:
                    last_station = ''
                    
                
                print('\n++++++++++++++++++++++++++++++++++++++\n' )
                
                    

                print("Hello UD")
            
            except  :
                
                   pass
           

            # disconnect the client
            #s.close()
            #===================================================================

            #counter += 1

            # Calculate the error in the x and y directions
            forward_error = self.fid_x - self.min_dist
            lateral_error = self.fid_y

            # Calculate the amount of turning needed towards the fiducial
            # atan2 works for any point on a circle (as opposed to atan)
            angular_error = math.atan2(self.fid_y, self.fid_x)     ######## Tmp:  This was modified. Husam multiplied by -1
            print("-----------------Hi there------------")
            print "Errors: forward %f lateral %f angular %f" % \
              (forward_error, lateral_error, degrees(angular_error))

            if self.got_fid:
                times_since_last_fid = 0
            else:
                times_since_last_fid += 1

            if forward_error > self.max_dist:
                print "Fiducial is too far away"
                linSpeed = 0
                angSpeed = 0
            # A fiducial was detected since last iteration of this loop
            elif self.got_fid and destFlag == False:
                # Set the turning speed based on the angular error
                # Add some damping based on the previous speed to smooth the motion 
                angSpeed = angular_error * self.angular_rate - angSpeed / 2.0
                # Make sure that the angular speed is within limits
                if angSpeed < -self.max_angular_rate:
                    angSpeed = -self.max_angular_rate
                if angSpeed > self.max_angular_rate:
                    angSpeed = self.max_angular_rate

                # Set the forward speed based distance
                linSpeed = forward_error * self.linear_rate
                # Make sure that the angular speed is within limits
                if linSpeed < -self.max_linear_rate:
                    linSpeed = -self.max_linear_rate
                if linSpeed > self.max_linear_rate:
                    linSpeed = self.max_linear_rate

            # ====================================
            elif self.got_fid and destFlag == True:
                linSpeed = 0
                angSpeed = 0
                


##                while 1:
##                    s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
##                    s.bind(('',12345))
##                    m=s.recv(1024)
##                    msg1=m.decode('utf-8')
##                    print('receieved',m)
##                    msg1 = msg.decode()
##                    print('Received:' + msg1)
##                    if msg1[0] == '0':
##                        destination = msg1[1:]
##                        print(destination)
##                    
##                    if msg1 == destination:
##                        print("Destination is reached!")
##                        # Keep turning in the same direction
##                        if angSpeed < 0:
##                            angSpeed = -self.lost_angular_rate
##                        elif angSpeed > 0:
##                            angSpeed = self.lost_angular_rate
##                        else:
##                            angSpeed = 0
##                            #print "Try keep rotating to refind fiducial: try# %d" % times_since_last_fid
##                        break
##                    
##                    else:
##                        print("Destination is not reached!")
##                        break
##                 
##                # disconnect the client
##                s.close()

                    

            # Hysteresis, don't immediately stop if the fiducial is lost
            elif not self.got_fid and times_since_last_fid < self.hysteresis_count:
                # Decrease the speed (assuming linear decay is <1)
                linSpeed *= self.linear_decay

            # Try to refind fiducial by rotating
            elif self.got_fid == False and times_since_last_fid < self.max_lost_count:
                # Stop moving forward
                linSpeed = 0
                angSpeed = 0  # added by Husameldin 
                # Keep turning in the same direction
                #if angSpeed < 0:
                #    angSpeed = -self.lost_angular_rate
                #elif angSpeed > 0:
                #    angSpeed = self.lost_angular_rate
                #else:
                #    angSpeed = 0
                #print "Try keep rotating to refind fiducial: try# %d" % times_since_last_fid
##                if self.fidID == 102:
##                    subprocess.call('python ~/catkin_ws/src/demos/fiducial_follow/nodes/switchbot.py cb:87:fa:95:c6:0b Bot Press', shell=True)
##                    self.fidID = 0
##                elif self.fidID == 103:
##                    subprocess.call('python ~/catkin_ws/src/demos/fiducial_follow/nodes/switchbot.py ca:be:97:b3:92:ee Bot Press', shell=True)
##                    self.fidID = 0
            else:
                angSpeed = 0
                linSpeed = 0

            print "Speeds: linear %f angular %f" % (linSpeed, angSpeed)

            # Create a Twist message from the velocities and publish it
            # Avoid sending repeated zero speed commands, so teleop
            # can work
            zeroSpeed = (angSpeed == 0 and linSpeed == 0)
            if not zeroSpeed:
                self.suppressCmd = False
            print "zero", zeroSpeed, self.suppressCmd
            if not self.suppressCmd:
                twist = Twist()
                twist.angular.z = angSpeed
                twist.linear.x = linSpeed
                self.cmdPub.publish(twist)
                if zeroSpeed:
                    self.suppressCmd = True

            # We already acted on the current fiducial
            self.got_fid = False
            rate.sleep()

            #sleep(1)

            #if counter > 100:
            #    subprocess.call('sudo -S python ~/catkin_ws/src/demos/fiducial_follow/nodes/switchbot.py cb:87:fa:95:c6:0b Bot Press', shell=True)
            #    break

            # disconnect the client
            #s.close()

if __name__ == "__main__":
    # Create an instance of our follow class
    node = Follow()
    # run it
    node.run()
