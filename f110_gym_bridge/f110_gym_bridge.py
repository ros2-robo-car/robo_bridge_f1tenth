import socket, struct, threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DEFAULT_HOST = 'localhost'
DEFAULT_PORT = 22200
RECEIVE_UNIT = 8192

PUBLISH_PERIOD = 1/80
ERRORSTR_FMT = "Connection Error with %s:%s: %s"

header_parser = struct.Struct('HH')

class F110GymBridge(Node):
    def __init__(self, cli_args=[]):
        super().__init__('f110_gym_bridge')
        self.host = DEFAULT_HOST
        self.port = DEFAULT_PORT
        if len(cli_args) >= 2:
            self.host = cli_args[0]
            self.port = cli_args[1]

        self.socket = None
        self.closedEvent = threading.Event()

        self.pubdata, self.publock = None, threading.Lock()
        self.publisher = self.create_publisher(String, "f110_recv", 10)
        self.subscriber = self.create_subscription(String, "f110_send", self.listen, 10)
        self.pub_interval = None
        
    def publish(self):
        self.publock.acquire()
        data = self.pubdata
        self.publock.release()

    def listen(self, msg):
        pass

    def close(self, e=None):
        if not e == None:
            self.get_logger().error(ERRORSTR_FMT.format(self.host, self.port, e))
        try: 
            self.socket.close()
        except:
            pass
        self.socket = None


    def connect(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.socket.connect((self.host, self.port))
        except Exception as e:
            self.close(e)
            return
        
        self.pub_interval = self.create_timer(PUBLISH_PERIOD, self.publish)

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

def main(args=None):
    rclpy.init(args=args)
    bridge = F110GymBridge()
    rclpy.spin(bridge)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
