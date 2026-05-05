import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class F110GymClientExample(Node):
    def __init__(self):
        self.recv_subscribe = None
        self.send_publisher = None


    def subscribe(self):
        self.recv_subscribe = self.create_subscription(String, "f110_recv", self.onRecv, 10)
        self.send_publisher = self.create_publisher(String, "f110_send", 10)
        # self.init_client = self.create_client(, 'init_sym')

        # self.req = None

    def onRecv(self, msg):
        pass