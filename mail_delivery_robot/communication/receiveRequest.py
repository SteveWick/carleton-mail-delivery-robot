#!/usr/bin/env python
# @author: Stephen Wicklund

# SUBSCRIBER:   NONE
# PUBLISHER:    Requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gql import gql, Client
from gql.transport.aiohttp import AIOHTTPTransport
import requests

database = AIOHTTPTransport(
    url="https://graphql.us.fauna.com/graphql",
    headers={'authorization': 'Bearer fnAEcsZAjRAAQFZKEY2ld2Lf-oxxylhdza1i8r9j',
    'Content-Type': 'application/json'})

getPendingRequests = gql(
"""
    query Test {
    requestsByState(state: "pending") {
        data {
        id,
        timestamp,
        destination,
        state,
        priority
        }
    }
    }
"""
)

class ReceiveRequest(Node):
  def __init__(self):
        super().__init__('receive_request')
        self.requestPublisher = self.create_publisher(String,'request',2)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.check_requests)
        client = Client(transport=database, fetch_schema_from_transport=True)

    def check_requests(self):
        if current_request == None:
            self.requestPublisher.publish(get_highest_priority_pending_requests()[0])

    def get_highest_priority_pending_requests(self):
        requests = client.execute(getOpenRequests)
        highestPriority = None
        for request in requests:
            if request.priority != None and request.priority > highestPriority:
                highestPriority = request.priority
        
        highestPriorityRequests = []
        for request in requests:
            if request.priority == highestPriority:
                highestPriorityRequests.add(request)

        return highestPriorityRequests


def main():
    rclpy.init()
    receive_request = ReceiveRequest()
    rclpy.spin(receive_request)

if __name__ == '__main__':
    main()