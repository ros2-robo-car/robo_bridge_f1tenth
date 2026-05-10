import queue, socket, struct, threading
import rclpy
from rclpy.node import Node
from f110_gym_bridge_interface.msg import Act, Obs, Recv, Status
from f110_gym_bridge_interface.srv import Initsim, Startsim
from .constants import MSGTYPE
from .packet_formatter import *

RECEIVE_UNIT = 8192
MIN_TIMESTEP = 0.01

header_parser = struct.Struct('!I')
type_parser = struct.Struct('!B')

class F110GymBridge(Node):
    def __init__(self):
        super().__init__('f110_gym_bridge')

        self.socket = None
        self.addr = None
        self.connected_event = threading.Event()
        self.sim_online_event = threading.Event()

        self.recv_thread = None
        self.recv_line = queue.Queue()
        self.recv_publisher = None
        self.recv_publisher_lock = threading.Lock()
        self.pub_interval = None
        self.send_subscriber = None
        self.send_subscriber_lock = threading.Lock()
        self.init_service = None

        self.get_logger().info("node f110_gym_bridge initialized.")
        self.init_service = self.create_service(Initsim, 'init_sim', self.initsim)
        self.start_service = self.create_service(Startsim, 'start_sim', self.startsim)
    
    def _flush_receive_line(self):
        with self.recv_line.mutex:
            while self.recv_line._qsize() > 0:
                self.recv_line._get()
    
    # callback of init_service
    def initsim(self, request: Initsim.Request, response: Initsim.Response):
        sim_status = Status()
        response = Initsim.Response(
            sim_status = sim_status
        )

        if self.connected_event.is_set():
            sim_status.status = Status.FAILURE
            sim_status.msg = "Bridge is using."
            self.get_logger().error("Request while bridge is using.")
            return response

        self.addr = (request.host, request.port)
        reqattr = {
            'timeout': request.timeout
        }

        self.get_logger().info(f"Init simulation to {self.addr[0]}:{self.addr[1]}")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if request.timeout > 0.0:
            self.socket.settimeout(request.timeout)
        else:
            self.socket.settimeout(None)
        try:
            self.socket.connect(self.addr)
            req_msg = pack(MSGTYPE.INIT_REQUEST, reqattr)
            req_msg = header_parser.pack(len(req_msg)) + req_msg
            self.socket.send(req_msg)

            res_msg = self.socket.recv(RECEIVE_UNIT)
            msgtype, resattr = unpack(res_msg[4:])
            if msgtype != MSGTYPE.INIT_RESPONSE:
                raise Exception(f"Expected START_RESPONSE, receive {msgtype.name} ({msgtype})")
            elif resattr['status'] >= Status.MAX:
                raise Exception(f"Invalid Status: {resattr['status']}")
            elif resattr['status'] >= Status.FAILURE:
                raise Exception(resattr['msg'])
        except Exception as e:
            sim_status.status = Status.FAILURE
            sim_status.msg = f"Init simulation Error: {e}"
            self.get_logger().error(sim_status.msg)
            self.close()
            return response
        
        msg = f"Init simulation. Sim Server is Ready: {resattr['msg']}"
        self.get_logger().info(msg)
        sim_status.status = resattr['status']
        sim_status.msg = msg
        self.connected_event.set()

        self.pub_interval = self.create_timer(max(resattr['timestep'], MIN_TIMESTEP), self.publish_flush)
        self.pub_interval.cancel()
        with self.recv_publisher_lock:
            self.recv_publisher = self.create_publisher(Recv, "f110_recv", 10)
        with self.send_subscriber_lock:
            self.send_subscriber = self.create_subscription(Act, "f110_send", self.listen, 10)

        return response
    
    # callback of start_service
    def startsim(self, request: Startsim.Request, response: Startsim.Response):
        sim_status = Status()
        response = Startsim.Response(
            sim_status = sim_status,
            timestep = request.timestep,
            flags = request.flags,
            map = request.map
        )

        if not self.connected_event.is_set():
            sim_status.status = Status.FAILURE
            sim_status.msg = f"Bridge has no connection. Init sim before start."
            self.get_logger().error(sim_status.msg)
            return response
        
        if self.sim_online_event.is_set():
            sim_status.status = Status.FAILURE
            sim_status.msg = f"Simulation is already running."
            self.get_logger().error(sim_status.msg)
            return response

        reqattr = {
            'timestep': request.timestep,
            'map': request.map,
            'flags': request.flags
        }
        
        try:
            req_msg = pack(MSGTYPE.START_REQUEST, reqattr)
            req_msg = header_parser.pack(len(req_msg)) + req_msg
            self.socket.send(req_msg)

            res_msg = self.socket.recv(RECEIVE_UNIT)
            msgtype, resattr = unpack(res_msg[4:])
            if msgtype != MSGTYPE.START_RESPONSE:
                raise Exception(f"Expected START_RESPONSE, receive {msgtype.name} ({msgtype})")
            elif resattr['status'] >= Status.MAX:
                raise Exception(f"Invalid Status: {resattr['status']}")
            elif resattr['status'] >= Status.FAILURE:
                raise Exception(resattr['msg'])
        except Exception as e:
            sim_status.status = Status.FAILURE
            sim_status.msg = f"Start simulation Error: {e}"
            self.get_logger().error(sim_status.msg)
            self.close()
            return response

        self.sim_online_event.set()
        self.pub_interval.reset()
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
    def publish_flush(self):
        with self.recv_line.mutex:
            while self.recv_line._qsize() > 0:
                self.publish(self.recv_line._get())
        
        if self.sim_online_event.is_set():
            self.pub_interval.cancel()

    def publish(self, recv_raw):
        if recv_raw == None:
            return
        
        if len(recv_raw) != struct_size(MSGTYPE.RECV):
            self.get_logger().error(f"Expected RECV size ({struct_size(MSGTYPE.RECV)}bytes), Receive {len(recv_raw)}bytes.")
            return

        msgtype, attr = unpack(recv_raw)
        if msgtype != MSGTYPE.RECV:
            self.get_logger().error(f"Wrong RECV type: {msgtype.name} ({msgtype})")
            return
        
        obs = Obs()
        obs.ego_idx, obs.scans, obs.collisions = attr['ego_idx'], attr['scans'], bool(attr['collisions'])
        obs.poses_x, obs.poses_y, obs.poses_theta = attr['poses_x'], attr['poses_y'], attr['poses_theta']
        obs.linear_vels_x, obs.linear_vels_y, obs.ang_vels_z = attr['linear_vels_x'], attr['linear_vels_y'], attr['ang_vels_z']

        recv = Recv()
        recv.obs = obs
        recv.elapsed_time = attr['elapsed_time']
        recv.sim_status = Status(status=attr['status'], msg=attr['msg'])
        
        self.recv_publisher_lock.acquire()
        if self.recv_publisher == None:
            self.recv_publisher_lock.release()
            return
        elif self.recv_publisher.get_subscription_count() == 0:
            self.recv_publisher_lock.release()
            self.get_logger().error(f"Disconnect with Client")
            self.close()
        else:
            self.recv_publisher.publish(recv)
            self.recv_publisher_lock.release()
        
        if recv.sim_status.status == Status.DONE:
            self.sim_online_event.clear()

    # callback of send_subscriber
    def listen(self, msg: Act):
        sendattr = {
            'steer': msg.steer,
            'speed': msg.speed
        }
        send_data = pack(MSGTYPE.SEND, sendattr)
        self.send(send_data)

    def log_error(self, sim_status: Status):
        verbose = 'Error'
        if sim_status.status == Status.FAILURE:
            verbose = 'Failure'
        self.get_logger().error(f"{verbose}: {sim_status.msg}")
        with self.recv_publisher_lock:
            if self.recv_publisher != None:
                sim_status.msg = f"{verbose}: {sim_status.msg}"
                self.recv_publisher.publish(Recv(sim_status=sim_status))

    def close(self):
        if not self.connected_event.is_set():
            return
        self.connected_event.clear()
        self.sim_online_event.clear()

        try: 
            self.socket.close()
        except:
            pass

        self.socket = None
        self.addr = None
        with self.recv_publisher_lock:
            if self.recv_publisher != None:
                self.recv_publisher.destroy()
                self.recv_publisher = None
        with self.send_subscriber_lock:
            if self.send_subscriber != None:
                self.send_subscriber.destroy()
                self.send_subscriber = None

        if self.recv_thread != None:
            if self.recv_thread != threading.current_thread():
                self.recv_thread.join()
            self.recv_thread = None

    def recvloop(self):
        while self.sim_online_event.is_set():
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
            return

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
            return

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
