import rclpy
from rclpy.node import Node

import argparse
from f110_gym_bridge_interface.msg import Act, Recv, Status
from f110_gym_bridge_interface.srv import Initsim

class F110GymClientExample(Node):
    def __init__(self, **kwargs):
        super().__init__('f110_gym_client')

    def request(self, **kwargs):
        self.init_client = self.create_client(Initsim, 'init_sym')
        if not self.init_client.wait_for_service(timeout_sec=3):
            self.get_logger().error('Service not available. Check whether service is on.')
            self.destroy_node()
            raise SystemExit

        req = Initsim.Request()
        for key in kwargs.keys():
            setattr(req, key, kwargs[key])
        return self.init_client.call_async(req)

    def run(self):
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
    for key in arg_defaults.keys(): 
        parser.add_argument(f'--{key}', default=argparse.SUPPRESS)
    for key in flag_const.keys():
        parser.add_argument(f'--{key}', action='store_true')

    parsed = parser.parse_args()
    args = {}
    for key in arg_defaults.keys(): 
        if not hasattr(parsed, key):
            print(f"use default value: {key} = {arg_defaults[key]}")
            args[key] = arg_defaults[key]
        else:
            args[key] = getattr(parsed, key)

    flags = 0
    for key in flag_const.keys():
        if getattr(parsed, key):
            flags |= flag_const[key]
    args['flags'] = flags


    try:
        rclpy.init()
        main_client = F110GymClientExample()

        future = main_client.request(**args)
        rclpy.spin_until_future_complete(main_client, future, timeout_sec=10)
        res = future.result()
        if res is None:
            main_client.get_logger().error('Can not receive response from sim server.')
            raise SystemExit
        elif res.sim_status.status == Status.ERROR:
            main_client.get_logger().error(res.sim_status.msg)
            raise SystemExit
        elif res.sim_status.status == Status.BUSY:
            main_client.get_logger().warn(res.sim_status.msg)
            raise SystemExit

        main_client.run()
        rclpy.spin(main_client)

    except KeyboardInterrupt:
        main_client.get_logger().info("Interrupted")
    except SystemExit:
        pass

    if rclpy.ok():
        main_client.destroy_node()
        rclpy.shutdown()
