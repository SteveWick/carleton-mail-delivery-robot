#!/usr/bin/env python
# @author: Stephen Wicklund

# SUBSCRIBER:   NONE
# PUBLISHER:    Requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
class ReceiveRequest(Node):
  def __init__(self):
        super().__init__('receive_request')
        self.requestPublisher = self.create_publisher(String,'request',2)
        timer_period = 10  # seconds
        self.timer = self.create_timer(timer_period, self.check_requests)

    def check_requests(self):
        #Check requests

        #TODO:
        request = requests.get('specific_url')
        if(request.status_code == 200):
            requestPublisher.publish(request)
        else:
            pass

def main():
    rclpy.init()
    receive_request = ReceiveRequest()
    rclpy.spin(receive_request)

if __name__ == '__main__':
    main()