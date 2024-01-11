#!/usr/bin/env python
from ament_index_python.packages import get_package_share_directory
from sys import byteorder
import sys
from array import array
from struct import pack
import rclpy

import pyaudio
import wave
import audioop
from rclpy.node import Node
from std_msgs.msg import String

THRESHOLD = 500
CHUNK_SIZE = 1024
FORMAT = pyaudio.paInt16

class RecordingPublisher(Node):
    # in the def init method i used device_name instaed of device_index,
    # for minimal error and device supporting reasons.
    def __init__(self, device_name):
#    def __init__(self, device_index):
        super().__init__('recording_publisher')
        self.publisher_ = self.create_publisher(String, 'recording', 10)
        self.listening = False
        self.create_timer(1, self.listen)  # check once per second
        # similarly, I used device_name instaed of device_index
        self.device_name = device_name
#        self.device_index = device_index
        # setting audio recording rate (here, used 44100 Hz instaed of 16 kHz)
        self.rate = 44100 # planned for 48000, if still error persists
#        self.rate = 16000 (original or given rate in template)

    def listen(self):
        if self.listening:
            return
        else:
            self.listening = True

        audio_path = get_package_share_directory('aisd_hearing')
        audio_path = audio_path + "/recordings"
        # made the necessary changes to find_device
        device = self.find_device(self.device_name)
#        device = self.find_device(self.device_index)
        current_time = str(self.get_clock().now().nanoseconds)
        audio_file = "{}/{}.wav".format(audio_path, current_time)

        try:
            self.record_to_file(audio_file, rate=self.rate, device=device)
        except Exception as e:
            print(f"Error recording audio: {e}")
            sys.exit(1)

        msg = String()
        msg.data = audio_file
        self.publisher_.publish(msg)
        self.listening = False

    ##############################
    # There are 6 methods taken from unr_deepspeech_client.py (as needed).
    # task is completed by pasting them here  here, also they added here
    # as a part of RecordingPublsher class definition.
    ##############################

    def record_to_file(self, path, rate, device):
        "Records from the microphone and outputs the resulting data to 'path'"
        sample_width, data = self.record(rate, device)

        if rate != 16000:
            data_16GHz = audioop.ratecv(data, sample_width, 1, rate, 16000, None)[0]
        else:
            data_16GHz = pack('<' + ('h'*len(data)), *data)

        wf = wave.open(path, 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(sample_width)
        wf.setframerate(16000)
        wf.writeframes(data_16GHz)
        wf.close()

    def record(self, rate, device):
        """
        Record a word or words from the microphone and
        return the data as an array of signed shorts.

        Normalizes the audio, trims silence from the
        start and end, and pads with 0.5 seconds of
        blank sound to make sure VLC et al can play
        it without getting chopped off.
        """
        p = pyaudio.PyAudio()
        ## the below one code added to existing one indicating device_name instaed of device_index.
        ### device = self.find_device(self.device_name) throws error so commenting for future ref
        ### below are existing code
        stream = p.open(format=FORMAT, channels=1, rate=rate,
                        input=True, output=True, frames_per_buffer=CHUNK_SIZE,
                        input_device_index=device)

        num_silent = 0
        snd_started = False
        r = array('h')

        while 1:
            snd_data = array('h', stream.read(CHUNK_SIZE, exception_on_overflow=False))
            if byteorder == 'big':
                snd_data.byteswap()
            r.extend(snd_data)

            silent = self.is_silent(snd_data)

            if silent and snd_started:
                num_silent += 1
            elif not silent and not snd_started:
                snd_started = True

            if snd_started and num_silent > 30:
                break

        sample_width = p.get_sample_size(FORMAT)
        stream.stop_stream()
        stream.close()
        p.terminate()

        r = self.normalize(r)
        r = self.trim(r)
        r = self.add_silence(r, 0.5, rate)
        return sample_width, r

    def find_device(self, device_name):
        p = pyaudio.PyAudio()

        # Find the device index based on the device name
        for i in range(p.get_device_count()):
            info = p.get_device_info_by_index(i)
            print(f"Device {i}: {info['name']} - {info['maxInputChannels']} input channels, {info['maxOutputChannels']} output channels")
            if info['name'] == device_name:
                return i

        print(f"Device {device_name} not found.")
        sys.exit(1)
#    def find_device(self, device_index):
#        p = pyaudio.PyAudio()
#        info = p.get_host_api_info_by_index(0)
#        numdevices = info.get('deviceCount')

#        if device_index == -1:
#            print("\nListing audio devices and exiting: ")
#            for i in range(0, numdevices):
#                if p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels') > 0:
#                    print("Input Device id ", i, " - ", p.get_device_info_by_host_api_device_index(0, i).get('name'))
#                    print("Supported sample rates: ", p.get_device_info_by_host_api_device_index(0, i).get('defaultSampleRate'))
#            sys.exit(0)
#        else:
#            try:
#                device = int(device_index)
#            except ValueError:
#                print("Invalid audio device id.")
#                print("usage: ros2 run aisd_hearing recording_publisher [audio_device_index]")
#                sys.exit(1)

        return device

    def is_silent(self, snd_data):
        "Returns 'True' if below the 'silent' threshold"
        return max(snd_data) < THRESHOLD

    def normalize(self, snd_data):
        "Average the volume out"
        MAXIMUM = 16384
        times = float(MAXIMUM) / max(abs(i) for i in snd_data)

        r = array('h')
        for i in snd_data:
            r.append(int(i * times))
        return r

    def trim(self, snd_data):
        "Trim the blank spots at the start and end"
        snd_started = False
        r = array('h')

        for i in snd_data:
            if not snd_started and abs(i) > THRESHOLD:
                snd_started = True
                r.append(i)

            elif snd_started:
                r.append(i)
        return r

    def add_silence(self, snd_data, seconds, rate):
        "Add silence to the start and end of 'snd_data' of length 'seconds' (float)"
        r = array('h', [0 for i in range(int(seconds * rate))])
        r.extend(snd_data)
        r.extend([0 for i in range(int(seconds * rate))])
        return r


def main(args=None):
    rclpy.init(args=args)
    #  replacing the device_index with the device name
    device_name = 'pulse'
#    device_index = 0
    recording_publisher = RecordingPublisher(device_name)
#    recording_publisher = RecordingPublisher(device_index)

    rclpy.spin(recording_publisher)
    recording_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
