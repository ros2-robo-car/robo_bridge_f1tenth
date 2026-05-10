import rclpy
from rclpy.node import Node
from collections.abc import Callable
import argparse, traceback, queue
from f110_gym_bridge_interface.msg import Act, Recv, Status
from f110_gym_bridge_interface.srv import Initsim

ARG_DEFAULTS = {
    'host': 'localhost',
    'port': 22200,
    'timestep': 0.025,
    'timeout': 30.0,
    'map': 'vegas'
}

FLAG_MASKS = {
    'async': Initsim.Request.FLAG_ASYNC
}

class F110GymClient(Node):
    _recv_callback: Callable[[Recv], None] = lambda recv: None
    _send_queue = queue.Queue()

    def __init__(self, recv_callback: Callable[[Recv], None]):
        """
        Create F110GymClient.
        
        Args:
            recv_callback ((Recv) -> None): function will be executed on receiving Recv data.
        """
        super().__init__('f110_gym_client')
        if recv_callback:
            self._recv_callback = recv_callback

    def send(self, msg: Act):
        """
        Send steering data to Sim server. (Thread safe)

        Args:
            msg (Act): Act data will be sent to Simulation. It contains steer, speed data.
        """
        if msg is not Act:
            raise TypeError('msg must be Act')
        self._send_queue.put(msg)

    def _request(self, **kwargs):
        self.init_client = self.create_client(Initsim, 'init_sym')
        if not self.init_client.wait_for_service(timeout_sec=3):
            self.get_logger().error('Service not available. Check whether service is on.')
            self.destroy_node()
            raise SystemExit

        req = Initsim.Request()
        for key in kwargs.keys():
            setattr(req, key, kwargs[key])
        return self.init_client.call_async(req)

    def _run(self, timestep):
        self.recv_subscribe = self.create_subscription(Recv, "f110_recv", self._on_recv, 10)
        self.send_publisher = self.create_publisher(Act, "f110_send", 10)
        self.create_timer(timestep, self._send)

    def _send(self):
        msg = None
        with self._send_queue.mutex:
            while self._send_queue._qsize() > 0:
                msg = self._send_queue._get()
        if msg != None:
            self.send_publisher.publish(msg)

    def _on_recv(self, recv_msg: Recv):
        if recv_msg.sim_status.status == Status.FAILURE:
            msg = f'status: [{recv_msg.sim_status.status}] {recv_msg.sim_status.msg}'
            self.get_logger().error(msg)
            self._close()
        elif recv_msg.sim_status.status == Status.ERROR:
            msg = f'status: [{recv_msg.sim_status.status}] {recv_msg.sim_status.msg}, '
            msg += f'poses: {recv_msg.obs.poses_x}, {recv_msg.obs.poses_y}'
            self.get_logger().error(msg)
        else:
            _guard_exception(self._recv_callback, recv_msg)
            
    def _close(self):
        self.recv_subscribe.destroy()
        self.send_publisher.destroy()
        self.destroy_node()
        raise SystemExit

def run_client_node(**kwargs):
    """
    Run F110GymClient. Before execution, Bridge node must be running.

    Args:
        host (str): Sim server's host name. Default is 'localhost'.
        port (int): Sim server's port. Default is 22200.
        timeout (float): Max wait time in seconds on connection. If value is 0 or negative, it will wait forever. Default is 30.
        timestep (float): Step interval of sim server in seconds. Default is 0.025.
        map (str): Name of map will be used in Simulation. It must be encoded ascii. Default is 'vegas'.
        async (bool): Flag that run on async mode. Default is False.
    """
    kwargs_checked = {}
    for key in ARG_DEFAULTS.keys(): 
        if not hasattr(kwargs, key):
            print(f"use default value: {key} = {ARG_DEFAULTS[key]}")
            kwargs_checked[key] = ARG_DEFAULTS[key]
        else:
            kwargs_checked[key] = getattr(kwargs, key)

    flags = 0
    for key in FLAG_MASKS.keys():
        if getattr(kwargs, key):
            flags |= FLAG_MASKS[key]
    kwargs_checked['flags'] = flags

    try:
        rclpy.init()
        main_client = F110GymClient()

        future = main_client._request(**kwargs_checked)
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

        main_client.get_logger().info(f'Sim Server is ready: {res.sim_status.msg}')
        main_client._run(res.timestep)
        rclpy.spin(main_client)

    except KeyboardInterrupt:
        main_client.get_logger().info("Interrupted")
    except SystemExit:
        pass

    if rclpy.ok():
        main_client.destroy_node()
        rclpy.shutdown()

def _guard_exception(func, *args, **kwargs):
    try:
        return func(*args, **kwargs)
    except KeyboardInterrupt:
        print('Interrupted')
        raise SystemExit
    except:
        traceback.print_exc()

def main():
    parser = argparse.ArgumentParser()
    for key in ARG_DEFAULTS.keys(): 
        parser.add_argument(f'--{key}', default=argparse.SUPPRESS)
    for key in FLAG_MASKS.keys():
        parser.add_argument(f'--{key}', action='store_true')

    parsed = parser.parse_args()
    run_client_node(parsed)
