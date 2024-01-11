#!/usr/bin/env python

from std_msgs.msg import String
from aisd_msgs.srv import Speak
import rclpy
from rclpy.node import Node

import wave
import struct

import sys
import glob
import os

import whisper

class WordsPublisher(Node):
    def __init__(self):
        super().__init__('words_publisher')
        self.publisher_ = self.create_publisher(String, 'words', 10)
        self.subscription = self.create_subscription(
            String,
            'recording',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.model = whisper.load_model("base")

    def listener_callback(self, msg):
        # prints logger information about receiving an audio message.
        self.get_logger().info('whisper is analysing: "%s"' % msg.data)
        # wave module is used to open the audio file
        audio_file = wave.open(msg.data)
        # getframerate() fuction is used in the audio file,
        # for better accuracy and analysing purposes.
        fs = audio_file.getframerate()
        # reads the frames from audio file.
        audio_string = audio_file.readframes(-1)
        audio = [struct.unpack("<h", audio_string[i:i+2])[0]
                 for i in range(0, len(audio_string), 2)]

        text = String()
        # mystt method is used to transcribe the audio and prints as message
        text.data = self.mystt(fs, msg.data)
        # prints the transcibed text with the help of mystt
        self.get_logger().info('I heard: "%s"' % text.data)
        # publish the transcribed text in topic
        self.publisher_.publish(text)

    def mystt(self,fs, audio):
        # STT model is used in transcribe method to convert  audio to text,
        #  and language english is opted here.
        transcription = self.model.transcribe(audio,fp16=False, language='English')
        # prints transcribed text in logger on screen
        self.get_logger().info('transcribe_result is: "%s"' % transcription['text'])
        return transcription['text'].strip()

def main(args=None): #added the content from the publisher template
    rclpy.init(args=args)
    words_publisher = WordsPublisher()
    rclpy.spin(words_publisher)
    words_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
