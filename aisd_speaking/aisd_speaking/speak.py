# speakingservice node
import io
from gtts import gTTS
from pydub import AudioSegment
from pydub.playback import play
from aisd_msgs.srv import Speak

import rclpy
from rclpy.node import Node

class SpeakingService(Node):

    def __init__(self):
        super().__init__('speaking_service')
        self.srv = self.create_service(Speak, 'speak', self.speak_callback)
        self.get_logger().info('Speaking Service Node Initialized')
    # #creating a call back function inside the class SpeakingService
    def speak_callback(self, request, response):
        # #BytesIO stream is used to hold binary data, inside class SpeakingService and assigns to a variable f
        with io.BytesIO() as f:
            # #  (gTTS(Google Text-to-Speech) opting english for language,stores the speech in a file-like object f
            gTTS(text=request.words, lang='en').write_to_fp(f)
            # Therefore, the  generated MP3 audio file are write to f
            # # seek() Python method is used to change(cursor position @ begining in  the current file position)
            f.seek(0)
            # song object is used to perform various audio processing tasks
            # Therefore,the audio data is stored in the format mp3 as specified
            song = AudioSegment.from_file(f, format="mp3")
            # this play the sudio file stored in song object of AudioSegment
            play(song)
            # indicates a successful completion of request in callbacl funtion
            response.response = "OK"
            # I used it to display status on the terminal
            self.get_logger().info('Speech generation and playback completed successfully')
        # feedback is received from the server after the service has been executed and terminates the callback function
        return response


def main():
    rclpy.init()
    speaking_service = SpeakingService()
    rclpy.spin(speaking_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
