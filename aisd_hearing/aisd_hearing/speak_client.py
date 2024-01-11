#!/usr/bin/env python

from std_msgs.msg import String
from aisd_msgs.srv import Speak
import rclpy
from rclpy.node import Node

class SpeakClient(Node):
    def __init__(self):
        super().__init__('speak_client')
        self.cli = self.create_client(Speak, 'speak')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Speak.Request()
        self.subscription = self.create_subscription(
            String,
            'words',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard tgk: "%s"' % msg.data)
        response = self.send_request('I heard: "%s"' % msg.data)
        self.get_logger().info('I got tgk: "%s"' % response)

    def send_request(self, text):
        self.req.words = text
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):

    rclpy.init(args=args)
    speak_client = SpeakClient()
    rclpy.spin(speak_client)
    speak_client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
