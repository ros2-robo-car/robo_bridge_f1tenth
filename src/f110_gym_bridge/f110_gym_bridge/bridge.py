import queue, socket, struct, threading
import rclpy
from rclpy.node import Node
from f110_gym_bridge_interface.msg import Act, Obs, Recv, Status
from f110_gym_bridge_interface.srv import Initsim
from .constants import MSGTYPE
from .packet_formatter import *

RECEIVE_UNIT = 8192
MIN_TIMESTEP = 0.01
TIMEOUT = 10

header_parser = struct.Struct('!I')
type_parser = struct.Struct('!B')

class F110GymBridge(Node):
    def __init__(self):
        super().__init__('f110_gym_bridge')
        # self.declare_parameter('host', 'localhost')
        # self.declare_parameter('port', 22200)

        # self.declare_parameter('timestep', 0.01)
        # self.declare_parameter('map', 'vegas')
        # self.declare_parameter('async_mode', False)

        self.socket = None
        self.addr = None
        self.closedEvent = threading.Event()

        self.pubdata, self.publock = None, threading.Lock()
        self.recv_thread = None
        self.recv_line = queue.Queue()
        self.recv_publisher = None
        self.pub_interval = None
        self.send_subscriber = None
        self.init_service = None

        self.get_logger().info("node f110_gym_bridge initialized.")
        self.init_service = self.create_service(Initsim, 'init_sym', self.initsim)

    def _last_from_receive_line(self):
        res = None
        with self.recv_line.mutex:
            while self.recv_line._qsize() > 0:
                res = self.recv_line._get()
        return res
    
    def _flush_receive_line(self):
        with self.recv_line.mutex:
            while self.recv_line._qsize() > 0:
                self.recv_line._get()
    
    # callback of init_service
    def initsim(self, request, response):
        self.closedEvent.clear()

        self.addr = (request.host, request.port)
        reqattr = {
            'timestep': request.timestep,
            'map': request.map,
            'flags': request.flags
        }

        sim_status = Status()
        response = Initsim.Response(
            sim_status = sim_status,
            timestep = request.timestep,
            flags = request.flags,
            map = request.map
        )
        

        self.get_logger().info(f"connecting to {self.addr[0]}:{self.addr[1]}")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(TIMEOUT)
        try:
            self.socket.connect(self.addr)
            send_msg = pack(MSGTYPE.REQUEST, reqattr)
            send_msg = header_parser.pack(len(send_msg)) + send_msg
            self.socket.send(send_msg)

            data = self.socket.recv(RECEIVE_UNIT)
            msgtype, resattr = unpack(data[4:])
            if msgtype != MSGTYPE.RESPONSE:
                raise Exception(f"Wrong Response type: {msgtype.name} ({msgtype})")
            elif resattr['status'] >= Status.MAX:
                raise Exception(f"Invalid Status: {resattr['status']}")
            elif resattr['status'] >= Status.FAILURE:
                raise Exception(f"Sim Server Response Error: {resattr['msg']}")
        except Exception as e:
            sim_status.status = Status.FAILURE
            sim_status.msg = f"Connection Error: {e}"
            self.log_error(sim_status)
            self.close()
            return response

        if resattr['status'] == Status.BUSY:
            msg = f"Sim Server is busy ({resattr['msg']}). try agian later"
            sim_status.status = Status.BUSY
            sim_status.msg = msg
            self.get_logger().warn(msg)
            self.close()
            return response

        self.pub_interval = self.create_timer(max(resattr['timestep'], MIN_TIMESTEP), self.publish)
        self.recv_publisher = self.create_publisher(Recv, "f110_recv", 10)
        self.send_subscriber = self.create_subscription(Act, "f110_send", self.listen, 10)

        self.recv_thread = threading.Thread(target=self.recvloop)
        self.recv_thread.start()

        msg = f"Sim Server is Ready: {resattr['msg']}"
        self.get_logger().info(msg)
        sim_status.status = resattr['status']
        sim_status.msg = msg
        response.timestep = resattr['timestep']
        response.flags = resattr['flags']
        response.map = resattr['map']

        return response

    # publish with recv_publisher
    def publish(self):

        data = self._last_from_receive_line()
        if data == None:
            return
        if len(data) != struct_size(MSGTYPE.RECV):
            self.get_logger().error(f"Expected RECV size ({struct_size(MSGTYPE.RECV)}bytes), Receive {len(data)}bytes.")
            return

        msgtype, attr = unpack(data)
        if msgtype != MSGTYPE.RECV:
            self.get_logger().error(f"Wrong RECV type: {msgtype.name} ({msgtype})")
            return
        
        obs = Obs()
        obs.ego_idx, obs.scans, obs.collisions = attr['ego_idx'], attr['scans'], attr['collisions']
        obs.poses_x, obs.poses_y, obs.poses_theta = attr['poses_x'], attr['poses_y'], attr['poses_theta']
        obs.linear_vels_x, obs.linear_vels_y, obs.ang_vels_z = attr['linear_vels_x'], attr['linear_vels_y'], attr['ang_vels_z']

        recv = Recv()
        recv.obs = obs
        recv.elapsed_time = attr['elapsed_time']
        recv.sim_status = Status(sim_status=attr['status'], msg=attr['msg'])
        
        self.recv_publisher.publish(recv)

    # callback of send_subscriber
    def listen(self, msg):
        pass

    def log_error(self, sim_status: Status):
        verbose = 'Error'
        if sim_status.status == Status.FAILURE:
            verbose = 'Failure'
        self.get_logger().error(f"{verbose} with {self.addr[0]}:{self.addr[1]}: {sim_status.msg}")
        if self.recv_publisher != None:
            sim_status.msg = f"{verbose}: {sim_status.msg}"
            self.recv_publisher.publish(Recv(sim_status=sim_status))

    def close(self):
        if self.closedEvent.is_set():
            return
        self.closedEvent.set()

        try: 
            self.socket.close()
        except:
            pass

        self.socket = None
        self.addr = None
        if self.recv_publisher != None: 
            self.recv_publisher.destroy()
            self.recv_publisher = None
        if self.send_subscriber != None:
            self.send_subscriber.destroy()
            self.send_subscriber = None

        if self.recv_thread != None:
            if self.recv_thread != threading.current_thread():
                self.recv_thread.join()
            self.recv_thread = None

    def recvloop(self):
        while not self.closedEvent.is_set():
            self.recv()

    def recv(self):
        if not self.is_socket_valid():
            self.close()
            return
        
        msg = b''
        try:
            recv = self.socket.recv(4)
            if len(recv) == 0:
                raise ConnectionError('Disconnect')
            msglen = header_parser.unpack(recv)[0]
            msg = self.socket.recv(msglen)
            self.recv_line.put(msg)
        except (ConnectionError, socket.error) as e:
            sim_status = Status(status=Status.FAILURE, msg=str(e))
            self.log_error(sim_status)
            self.close()
            return
        except Exception as e:
            sim_status = Status(status=Status.ERROR, msg=str(e))
            self.log_error(sim_status)
            return
    
    def send(self, data):
        if not self.is_socket_valid():
            self.close()
            return 0

        header = header_parser.pack(len(data))
        msg = header + data
        try:
            return self.socket.send(msg)
        except (ConnectionError, socket.error) as e:
            sim_status = Status(status=Status.FAILURE, msg=str(e))
            self.log_error(sim_status)
            self.close()
            return
        except Exception as e:
            sim_status = Status(status=Status.ERROR, msg=str(e))
            self.log_error(sim_status)
            return 0

    def is_socket_valid(self):
        return type(self.socket) == socket.socket

def main(args=None):
    try:
        rclpy.init(args=args)
        main_bridge = F110GymBridge()
        rclpy.spin(main_bridge)
    except KeyboardInterrupt:
        main_bridge.get_logger().info("Interrupted")
        main_bridge.close()

    if rclpy.ok():
        main_bridge.destroy_node()
        rclpy.shutdown()
