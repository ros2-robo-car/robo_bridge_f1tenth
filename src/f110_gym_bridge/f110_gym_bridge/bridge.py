import socket, struct, threading
import rclpy
from rclpy.node import Node
from f110_gym_bridge_interface.msg import Act, Recv
from f110_gym_bridge_interface.srv import Initsim
from .constants import MSGTYPE, STATUS
from .packet_formatter import pack, unpack

RECEIVE_UNIT = 8192
MIN_TIMESTEP = 0.01
TIMEOUT = 10

header_parser = struct.Struct('!I')

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
        self.recv_publisher = None
        self.pub_interval = None
        self.send_subscriber = None
        self.init_service = None

        self.get_logger().info("node f110_gym_bridge initialized.")
        self.init_service = self.create_service(Initsim, 'init_sym', self.initsim)
    
    # callback of init_service
    def initsim(self, request, response):
        self.closedEvent.clear()
        self.addr = (request.host, request.port)
        reqattr = {
            'timestep': request.timestep,
            'map': request.map,
            'flags': request.flags
        }
        err_response = {
            'status': int(STATUS.ERROR),
            'msg': '',
            'timestep': request.timestep,
            'flags': request.flags,
            'map': request.map
        }

        self.get_logger().info(f"connecting to {self.addr[0]}:{self.addr[1]}")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(TIMEOUT)
        try:
            self.socket.connect(self.addr)
            self.socket.send(pack(MSGTYPE.REQUEST, reqattr))

            data = self.socket.recv(RECEIVE_UNIT)
            msgtype, resattr = unpack(data[4:])
            if msgtype != MSGTYPE.RESPONSE:
                raise Exception(f"Wrong Response type: {msgtype}")
            if resattr['status'] == STATUS.ERROR:
                raise Exception(f"Sim Server Response Error: {resattr['msg']}")
        except Exception as e:
            err_response['msg'] = f"Connection Error: {e}"
            for key in err_response.keys():
                setattr(response, key, err_response[key])
            self.close(e)
            return response

        if resattr['status'] == STATUS.BUSY:
            msg = f"server is busy ({resattr['msg']}). try agian later"
            err_response['msg'] = msg
            err_response['status'] = int(STATUS.BUSY)
            for key in err_response.keys():
                setattr(response, key, err_response[key])
            self.get_logger().warn(msg)
            self.close()
            return response

        for key in resattr.keys():
            setattr(response, key, resattr[key])
        response.status = int(response.status)

        self.pub_interval = self.create_timer(max(resattr['timestep'], MIN_TIMESTEP), self.publish)
        self.recv_publisher = self.create_publisher(Recv, "f110_recv", 10)
        self.send_subscriber = self.create_subscription(Act, "f110_send", self.listen, 10)

        self.recv_thread = threading.Thread(target=self.recvloop)
        self.recv_thread.start()

        return response

    # publish with recv_publisher
    def publish(self):
        self.publock.acquire()
        data = self.pubdata
        self.publock.release()

    # callback of send_subscriber
    def listen(self, msg):
        pass

    def close(self, e=None):
        # is it in worker thread?
        if self.closedEvent.is_set():
            return

        if not e == None:
            self.get_logger().error(f"Connection Error with {self.addr[0]}:{self.addr[1]}: {e}")

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
            self.recv_thread.join()
            self.recv_thread = None

        self.closedEvent.set()

    def recvloop(self):
        while not self.closedEvent.is_set():
            self.recv()

    def recv(self):
        if not self.is_socket_valid():
            self.close()
            return
        
        msg = b''
        try:
            recv = self.socket.recv(RECEIVE_UNIT)
            msg += recv
            while len(recv) > 0:
                recv = self.socket.recv(RECEIVE_UNIT)
                msg += recv
        except Exception as e:
            self.close(e)
            return

        cur = 0
        lastdata = None
        while cur < len(msg):
            msglen = header_parser.unpack(msg[cur:cur+4])
            if cur + msglen + 4 > len(msg): break
            data = msg[ cur + 4 : cur + msglen + 4 ]
            lastdata = data
            cur += msglen + 4
        
        self.publock.acquire()
        self.pubdata = lastdata
        self.publock.release()
    
    def send(self, data):
        if not self.is_socket_valid():
            self.close()
            return 0

        header = header_parser.pack(len(data))
        msg = header + data
        try:
            return self.socket.send(msg)
        except Exception as e:
            self.close(e)
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
        return

    if rclpy.ok():
        main_bridge.destroy_node()
        rclpy.shutdown()
