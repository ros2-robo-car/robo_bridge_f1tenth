import rclpy
from rclpy.node import Node

import argparse
from constants import STATUS
from f110_gym_bridge_interface.msg import Act, Recv
from f110_gym_bridge_interface.srv import Initsim

class F110GymClientExample(Node):
    def __init__(self, **kwargs):
        
        self.init_client = self.create_client(Initsim, 'init_sym')
        if not self.init_client.wait_for_service(timeout_sec=3):
            self.get_logger().info('service not available. check whether service is on.')

        req = Initsim.Request()
        for key, value in kwargs:
            req[key] = value
        res = self.init_client.call(req)

        if res['status'] == STATUS.ERROR:
            self.get_logger().error(res['msg'])
        elif res['status'] == STATUS.BUSY:
            self.get_logger().warn(res['msg'])

        self.recv_subscribe = self.create_subscription(Recv, "f110_recv", self.onRecv, 10)
        self.send_publisher = self.create_publisher(Act, "f110_send", 10)

    def subscribe(self):
        pass

    def onRecv(self, msg):
        pass

def main():
    arg_defaults = {
        'host': 'localhost',
        'port': 22200,
        'timestep': 0.025,
        'map': 'vegas'
    }

    flag_const = {
        'async': 1 << 0
    }

    parser = argparse.ArgumentParser()
    for key, _ in arg_defaults: 
        parser.add_argument(f'--{key}', default=argparse.SUPPRESS)
    for key, _ in flag_const:
        parser.add_argument(f'--{key}', action='store_true')

    parsed = parser.parse_args()
    args = {}
    for key, value in arg_defaults: 
        if not hasattr(parsed, key):
            print(f"use default value: {key} = {value}")
            args[key] = value
        else:
            args[key] = parsed[key]

    flags = 0
    for key, value in flag_const:
        if parsed[key]:
            flags |= flag_const[key]
    args['flags'] = flags


    try:
        rclpy.init()
        main_client = F110GymClientExample(args)
        rclpy.spin(main_client)
    except KeyboardInterrupt:
        main_client.get_logger().info("Interrupted")
    rclpy.shutdown()