import rclpy
from rclpy.node import Node
from collections.abc import Callable
from threading import Event
import argparse, traceback, queue
from f110_gym_bridge_interface.msg import Act, Recv, Status
from f110_gym_bridge_interface.srv import Initsim, Startsim

INIT_ARGS_DEFAULT = {
    'host': 'localhost',
    'port': 22200,
    'timeout': 30.0
}

START_ARGS_DEFAULT = {
    'timestep': 0.025,
}

START_FLAG_MASKS = {
    'async': Initsim.Request.FLAG_ASYNC
}

class F110GymClient(Node):
    timeout = 30
    _publish_interval = 0.01
    _recv_callback: Callable[[Recv], None] = lambda recv: None
    _sim_online_event = Event()
    _sim_done_future = None
    _send_queue = queue.Queue()

    def __init__(self, recv_callback: Callable[[Recv], None]):
        """
        Create F110GymClient.

        Args:
            recv_callback ((Recv) -> None): function will be exectued on receiving Recv data.

        Returns:
            Created F110GymClient Node
        """
        super().__init__('f110_gym_client')

        if type(recv_callback) != Callable[[Recv], None]:
            raise TypeError('recv_callback must be Callable[[Recv], None]')
        self._recv_callback = recv_callback

        self.init_client = self.create_client(Initsim, 'init_sim')
        self.start_client = self.create_client(Startsim, 'start_sim')
        self.recv_subscribe = self.create_subscription(Recv, "f110_recv", self._on_recv, 10)
        self.send_publisher = self.create_publisher(Act, "f110_send", 10)

    def request_init(self, **kwargs):
        """
        Request to Init Simulation. Before execution, Bridge node must be running.
        
        Args:
            host (str): Sim server's host name. Default is 'localhost'.
            port (int): Sim server's port. Default is 22200.
            timeout (float): Max wait time in seconds on connection. If value is 0 or negative, it will wait forever. Default is 30.

        Returns:
            Future will be done when receiving response.
        """
        self.timeout = getattr(kwargs, 'timeout', 30)
        return self._request(INIT_ARGS_DEFAULT, [], self.init_client, Initsim.Request, kwargs)
    
    def response_init(self, res:Initsim.Response):
        try:
            self._assert_response_error(res)
            self.get_logger().info(f'Sim server is initialized: {res.sim_status.msg}')
        except SystemExit as e:
            self.get_logger().error(f'Failed to Initialized: {res.sim_status.msg}')
            raise e
    
    def request_start(self, **kwargs):
        """
        Request to Start Simulation. Before execution, Simulation should be initialized.

        Args:
            timestep (float): Step interval of sim server in seconds. Default is 0.025.
            map (str): Name of map will be used in Simulation. It must be encoded ascii. Default is 'vegas'.
            async (bool): Flag that run on async mode. Default is False.

        Returns:
            Future will be done when receiving response.
        """
        return self._request(START_ARGS_DEFAULT, START_FLAG_MASKS, self.start_client, Startsim.Request, kwargs)
    
    def response_start(self, res:Startsim.Response):
        try:
            self._assert_response_error(res)

            self._publish_interval = res.timestep
            self.get_logger().info(f'Sim server is Ready: {res.sim_status.msg}')
        except SystemExit as e:
            self.get_logger().error(f'Failed to Start: {res.sim_status.msg}')
            raise e
    
    def run(self):
        """
        Returns:
            Futuer will be done when simulation is done. It's result is latest Recv data.
        """
        self._sim_done_future = rclpy.Future()
        self._publish_timer = self.create_timer(self._publish_interval, self._send)
        return self._sim_done_future

    def send(self, msg: Act):
        """
        Send steering data to Sim server. (Thread safe)

        Args:
            msg (Act): Act data will be sent to Simulation. It contains steer, speed data.
        """
        if type(msg) != Act:
            raise TypeError('msg must be Act')
        if not self._sim_online_event.is_set():
            return
        self._send_queue.put(msg)

    def _request(self, args, flags, client, creator, kwargs):
        kwargs_checked = {}
        for key in args.keys(): 
            if not hasattr(kwargs, key):
                print(f"use default value: {key} = {args[key]}")
                kwargs_checked[key] = args[key]
            else:
                kwargs_checked[key] = getattr(kwargs, key)

        _flags = 0
        for key in flags.keys():
            if getattr(kwargs, key):
                _flags |= flags[key]
        kwargs_checked['flags'] = _flags

        if not client.wait_for_service(timeout_sec=3):
            self.get_logger().error('Service not available. Check whether service is on.')
            self.destroy_node()
            raise SystemExit

        req = creator()
        for key in kwargs.keys():
            setattr(req, key, kwargs[key])
        return self.client.call_async(req)

    def _assert_response_error(self, res):
        if res is None:
            raise SystemExit('Can not receive response from sim server.')
        elif res.sim_status.status == Status.FAILURE:
            raise SystemExit(res.sim_status.msg)
        elif res.sim_status.status == Status.ERROR:
            raise SystemExit(res.sim_status.msg)
        elif res.sim_status.status == Status.BUSY:
            raise SystemExit(res.sim_status.msg)

    def _send(self):
        msg = None
        with self._send_queue.mutex:
            while self._send_queue._qsize() > 0:
                msg = self._send_queue._get()
        if msg != None:
            self.send_publisher.publish(msg)

    def _on_recv(self, recv_msg: Recv):
        if recv_msg.sim_status.status == Status.FAILURE:
            raise SystemExit(f"FAILURE: {recv_msg.sim_status.msg}")
        elif recv_msg.sim_status.status == Status.ERROR:
            self.get_logger().error(f"ERROR: {recv_msg.sim_status.msg}")
        elif recv_msg.sim_status.status == Status.DONE:
            self._publish_timer.destroy()
            self._sim_online_event.clear()
            self._sim_done_future.set_result(recv_msg)
        else:
            try:
                self._recv_callback(recv_msg)
            except:
                traceback.print_exc()

def main():
    parser = argparse.ArgumentParser()
    for key in INIT_ARGS_DEFAULT.keys(): 
        parser.add_argument(f'--{key}', default=argparse.SUPPRESS)
    for key in START_FLAG_MASKS.keys():
        parser.add_argument(f'--{key}', action='store_true')
    parsed = parser.parse_args()
    kwargs = vars(parsed)

    try:
        rclpy.init()
        main_client = F110GymClient(lambda x: None)

        future = main_client.request_init(**kwargs)
        rclpy.spin_until_future_complete(main_client, future, timeout_sec=main_client.timeout)
        main_client.response_init(future.result())

        future = main_client.request_start(**kwargs)
        rclpy.spin_until_future_complete(main_client, future, timeout_sec=main_client.timeout)
        main_client.response_start(future.result())

        future = main_client.run()
        rclpy.spin_until_future_complete(main_client, future)

    except KeyboardInterrupt:
        main_client.get_logger().info("Interrupted")
    except SystemExit:
        pass
    finally:
        if rclpy.ok():
            main_client.destroy_node()
            rclpy.shutdown()
