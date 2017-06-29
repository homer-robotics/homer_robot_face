#!/usr/bin/env python
import sys, os

import rospy
import urllib2

from std_msgs.msg import String, Empty

from dynamic_reconfigure.server import Server
from homer_tts.cfg import MaryTTSConfig

class MaryTTSSpeak:

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
    processed_text = urllib2.quote(text)

    bool_to_str = ["off", "on"]
    effect_robot = rospy.get_param('/mary_tts/robot', False) 
    effect_stadium = rospy.get_param('/mary_tts/stadium', False)
    effect_whisper = rospy.get_param('/mary_tts/whisper', False)
    locale = rospy.get_param('/mary_tts/locale', 'en_US')
    voice = rospy.get_param('/mary_tts/voice', 'cmu_slt').replace("_","-")

    url = "http://localhost:59125/process?INPUT_TEXT=%s&INPUT_TYPE=TEXT&"\
            "OUTPUT_TYPE=AUDIO&AUDIO=WAVE_FILE"\
            "&LOCALE=%s"\
            "&effect_robot_selected=%s" \
            "&effect_stadium_selected=%s" \
            "&effect_whisper_selected=%s" \
            "&VOICE=%s" \
            % (processed_text, 
                    locale, 
                    bool_to_str[effect_robot], 
                    bool_to_str[effect_stadium], 
                    bool_to_str[effect_whisper], 
                    voice)
    os.system("curl -s \"" + url + "\" | aplay -q")

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

rospy.init_node('mary_tts')
mary = MaryTTSSpeak()
rospy.loginfo("mary tts node. This node assumes that the mary httpserver is runing on port 59125")

while not rospy.is_shutdown():
    mary.handle_queue()
    rospy.sleep(0.2)

