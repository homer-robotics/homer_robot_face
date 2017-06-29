#!/usr/bin/env python
import sys, os
from subprocess import call

import roslib
import rospy

import pyaudio  
import urllib2
import wave  

from std_msgs.msg import String, Empty
from dynamic_reconfigure.server import Server
# from homer_tts.cfg import MaryTTSConfig

import rospy

from dynamic_reconfigure.server import Server
# from dynamic_tutorials.cfg import TutorialsConfig
# from dynamic_tutorials.cfg import MaryTTSConfig
from homer_tts.cfg import MaryTTSConfig

class MarryTTsSpeak:


  def __init__(self):
    self.text_out_sub = rospy.Subscriber("/robot_face/text_out", String,
            self.speak_callback)
    self.talking_finished_pub = rospy.Publisher("/robot_face/talking_finished", 
            String, queue_size=0)

    self.dynamic_reconfigure_server = Server(MaryTTSConfig,
            self.dynamic_reconfigure_callback)
    self.text_queue = []

  def dynamic_reconfigure_callback(self, config, level):
    rospy.loginfo(config)
    return config

  def retrive_wav(self, filename, text):
    processed_text = urllib2.quote(text)
    print(processed_text)
    os.system("pico2wave \""+processed_text+"\" --lang=en-US --wave="+filename)


  def play_wav_file(self, filename):
    INPUT_FRAMES_PER_BLOCK = 1024
    wf = wave.open(filename, 'r')
    pa = pyaudio.PyAudio()
    stream = pa.open(format=pa.get_format_from_width(wf.getsampwidth()),
                     channels=wf.getnchannels(),
                     rate=wf.getframerate(),
                     output=True)
    data = wf.readframes(INPUT_FRAMES_PER_BLOCK)
    while data != '':
        stream.write(data)
        data = wf.readframes(INPUT_FRAMES_PER_BLOCK)
    stream.stop_stream()
    stream.close()
    pa.terminate()

  def speak_callback(self, data):
      self.text_queue.append(data.data)
      while self.text_queue[0] != data.data:
          rospy.sleep(0.5)
      self.retrive_wav("/tmp/lisa_speak", self.text_queue[0])
      os.system("amixer set Capture 0%")
      self.play_wav_file("/tmp/lisa_speak")
      msg = String()
      os.system("amixer set Capture 100%")
      self.text_queue.pop(0)
      self.talking_finished_pub.publish(msg)

rospy.init_node('pico_tts')
MarryTTsSpeak()
rospy.sleep(1.0)
rospy.loginfo("pico tts node is now running")
rospy.spin()
