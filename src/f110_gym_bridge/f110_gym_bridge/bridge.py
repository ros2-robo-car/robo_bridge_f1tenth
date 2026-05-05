import socket, struct, threading
import rclpy
from rclpy.node import Node
from f110_gym_bridge_interface.msg import Act, Recv
from f110_gym_bridge_interface.srv import Initsim
from constants import MSGTYPE, STATUS
import packet_formatter as formatter

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
        self.closedEvent = threading.Event()

        self.pubdata, self.publock = None, threading.Lock()
        self.recv_publisher = None
        self.pub_interval = None
        self.send_subscriber = None
        self.init_service = None

        self.get_logger().info("node f110_gym_bridge initialized.")
        self.init_service = self.create_service(Initsim, 'init_sym', self.initsim)
    
    # callback of init_service
    def initsim(self, request: dict, response: dict):
        self.closedEvent.clear()
        host = request.get('host', 'localhost')
        port = request.get('port', 22200)
        reqattr = {
            'timestep': request.get('timestep', 0.025),
            'map': request.get('map', 'vegas'),
            'flags': request.get('flags', 0)
        }
        err_response = {
            'status': STATUS.ERROR,
            'msg': '',
            'timestep': 0,
            'flags': 0
        }

        self.get_logger().info(f"connecting to {host}:{port}")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(TIMEOUT)
        try:
            self.socket.connect((self.host, self.port))
            self.socket.send(formatter.pack(MSGTYPE.REQUEST, reqattr))

            data = self.socket.recv(RECEIVE_UNIT)
            msgtype, resattr = formatter.unpack(data[4:])
            if msgtype != MSGTYPE.RESPONSE:
                raise Exception(f"Wrong Response type: {msgtype}")
            if resattr['status'] == STATUS.ERROR:
                raise Exception(f"Sim Server Response Error: {resattr['msg']}")
        except Exception as e:
            err_response['msg'] = f"Connection Error: {e}"
            for key, value in err_response:
                response[key] = value
            self.close(e)
            return

        if resattr['status'] == STATUS.BUSY:
            msg = f"server is busy ({resattr['msg']}). try agian later"
            err_response['msg'] = msg
            err_response['status'] = STATUS.BUSY
            for key, value in err_response:
                response[key] = value
            self.get_logger().warn(msg)
            self.close()
            return

        for key, value in resattr:
            response[key] = value

        self.pub_interval = self.create_timer(max(resattr['timestep'], MIN_TIMESTEP), self.publish)
        self.recv_publisher = self.create_publisher(Recv, "f110_recv", 10)
        self.send_subscriber = self.create_subscription(Act, "f110_send", self.listen, 10)

        recv_thread = threading.Thread(target=self.recvloop)
        recv_thread.start()

    # publish with recv_publisher
    def publish(self):
        self.publock.acquire()
        data = self.pubdata
        self.publock.release()

    # callback of send_subscriber
    def listen(self, msg):
        pass

    def close(self, e=None):
        if not e == None:
            addr = self.socket.getpeername()
            self.get_logger().error(f"Connection Error with {addr[0]}:{addr[1]}: {e}")
        try: 
            self.socket.close()
        except:
            pass
        self.closedEvent.set()
        self.socket = None
        if self.recv_publisher != None: 
            self.recv_publisher.destroy()
            self.recv_publisher = None
        if self.send_subscriber != None:
            self.send_subscriber.destroy()
            self.send_subscriber = None

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
    rclpy.shutdown()