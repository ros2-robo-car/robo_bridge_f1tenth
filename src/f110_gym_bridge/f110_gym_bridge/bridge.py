import socket, struct, threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .constants import MSGTYPE, STATUS, sock_format

RECEIVE_UNIT = 8192

PUBLISH_PERIOD = 1/80
ERR_CONNECT_FMT = "Connection Error with %s:%s: %s"

header_parser = struct.Struct('!HH')

class F110GymBridge(Node):
    def __init__(self):
        super().__init__('f110_gym_bridge')
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 22200)

        self.declare_parameter('timestep', 0.01)
        self.declare_parameter('map', 'vegas')
        self.declare_parameter('async_mode', False)

        self.socket = None
        self.closedEvent = threading.Event()

        self.pubdata, self.publock = None, threading.Lock()
        self.recv_publisher = None
        self.pub_interval = None
        self.send_subscriber = None
        self.init_service = None

        self.get_logger().info("node f110_gym_bridge initialized.")
    
    # publish with recv_publisher
    def publish(self):
        self.publock.acquire()
        data = self.pubdata
        self.publock.release()

    # callback of send_subscriber
    def listen(self, msg):
        pass

    # callback of init_service
    def initsim(self, request, response):
        pass

    def close(self, e=None):
        if not e == None:
            addr = self.socket.getpeername()
            self.get_logger().error(ERR_CONNECT_FMT.format(addr[0], addr[1], e))
        try: 
            self.socket.close()
        except:
            pass
        self.closedEvent.set()
        self.socket = None
        self.recv_publisher.destroy()
        self.send_subscriber.destroy()

    def connect(self):
        self.closedEvent.clear()
        host = self.get_parameter('host').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        
        self.get_logger().info(f"connecting to {host}:{port}")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.socket.connect((self.host, self.port))
        except Exception as e:
            self.close(e)
            return
        
        self.pub_interval = self.create_timer(PUBLISH_PERIOD, self.publish)
        self.recv_publisher = self.create_publisher(String, "f110_recv", 10)
        self.send_subscriber = self.create_subscription(String, "f110_send", self.listen, 10)
        # self.init_service = self.create_service(, 'init_sym', self.initsim)

        recv_thread = threading.Thread(target=self.recvloop)
        recv_thread.start()

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
            msgtype, msglen = header_parser.unpack(msg[cur:cur+2])
            if cur + msglen + 2 > len(msg): break
            data = msg[ cur + 2 : cur + msglen + 2 ]
            lastdata = (msgtype, data)
            cur += msglen + 2
        
        self.publock.acquire()
        self.pubdata = lastdata
        self.publock.release()
    
    def send(self, type, data):
        if not self.is_socket_valid():
            self.close()
            return 0

        header = header_parser.pack(type, len(data))
        msg = header + data
        try:
            return self.socket.send(msg)
        except Exception as e:
            self.close(e)
            return 0

    def is_socket_valid(self):
        return type(self.socket) == socket.socket

main_bridge = None

def main(args=None):
    rclpy.init(args=args)
    main_bridge = F110GymBridge()
    rclpy.spin(main_bridge)
    rclpy.shutdown()