#!/usr/bin/env python
import sys, os

import rospy

from std_msgs.msg import String, Empty

from dynamic_reconfigure.server import Server
from homer_tts.cfg import MaryTTSConfig

class FliteSpeak:

  def __init__(self):
    self.text_out_sub = rospy.Subscriber("/robot_face/text_out", String, 
            self.speak_callback)
    self.talking_finished_pub = rospy.Publisher("/robot_face/talking_finished", 
            String, queue_size=0)

    self.dynamic_reconfigure_server = Server(MaryTTSConfig, 
            self.dynamic_reconfigure_callback)
    self.text_queue = []
    self.muted = True

  def dynamic_reconfigure_callback(self, config, level):
    rospy.logdebug(config)
    return config

  def speak(self, text):
    os.system("flite -t \""+ text +"\" -voice slt")

  def speak_callback(self, data):
      self.text_queue.append(data.data)

  def handle_queue(self):
      if len(self.text_queue): 
          os.system("amixer -q set Capture nocap")
          self.muted = True
          if self.text_queue[0].strip() != "":
              self.speak(self.text_queue[0])
          msg = String()
          msg.data = self.text_queue[0]
          self.talking_finished_pub.publish(msg)
          self.text_queue.pop(0)

      else:
          if self.muted:
              os.system("amixer -q set Capture 100%")
              os.system("amixer -q set Capture cap")
              self.muted = False

rospy.init_node('flite_tts')
flite = FliteSpeak()
rospy.loginfo("flite tts node is now running")

while not rospy.is_shutdown():
    flite.handle_queue()
    rospy.sleep(0.2)

