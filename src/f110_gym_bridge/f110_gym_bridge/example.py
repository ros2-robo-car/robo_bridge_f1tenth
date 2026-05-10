import argparse, threading, time
import f110_gym_bridge.example_keyboard as keyboard
from f110_gym_bridge_interface import *
from f110_gym_bridge.client import *

key_state_lock = threading.Lock()
key_state = {
    "up": 0,
    "down": 0,
    "right": 0,
    "left": 0,
}
terminated_event = threading.Event()
INTERVAL = 1/40


def key_event_callback(e):
    key_state_lock.acquire()
    key_state[e.name] = True if e.event_type == "down" else False
    key_state_lock.release()

last_receive_time = 0
last_status = Status.DONE
PRINT_RECV_INTERVAL = 3
def on_receive(msg: Recv):
    global last_status, last_receive_time
    global PRINT_RECV_INTERVAL

    if msg.sim_status.status != last_status:
        last_status = msg.sim_status.status
        print(f"[{time.time()}] Status: {last_status}")

    cur_time = time.time()
    if cur_time - last_receive_time > PRINT_RECV_INTERVAL:
        last_receive_time = cur_time
        print(f"[{time.time()}] x: {msg.obs.poses_x}, y:{msg.obs.poses_y}")

client = None
PRESS_TIME = 3
def update():
    global client
    steer, speed = 0., 0.
    while not terminated_event.is_set():
        curTime = time.time()
        key_state_lock.acquire()
        key_up = (curTime - key_state["up"] < PRESS_TIME)
        key_left = (curTime - key_state["left"] < PRESS_TIME)
        key_right = (curTime - key_state["right"] < PRESS_TIME)
        key_state_lock.release()
        speed = 5.0 if key_up else 0.0
        steer = (key_left - key_right) * 1.0

        if client != None:
            act = Act()
            act.steer = steer
            act.speed = speed
            client.send(act)

        elapsedTime = time.time() - curTime
        if elapsedTime < INTERVAL:
            time.sleep(INTERVAL - elapsedTime)

def readkey():
    with keyboard.ConsoleKeyReader() as keyReader:
        while not terminated_event.is_set():
            c = keyReader.readone()
            cur_time = time.time()
            with key_state_lock:
                if c == keyboard.KEYCODE_UP: key_state["up"] = cur_time
                elif c == keyboard.KEYCODE_DOWN: key_state["down"] = cur_time
                elif c == keyboard.KEYCODE_LEFT: key_state["left"] = cur_time
                elif c == keyboard.KEYCODE_RIGHT: key_state["right"] = cur_time
                elif c == 'w': key_state["up"] = cur_time
                elif c == 's': key_state["down"] = cur_time
                elif c == 'a': key_state["left"] = cur_time
                elif c == 'd': key_state["right"] = cur_time

            elapsedTime = time.time() - cur_time
            if elapsedTime < INTERVAL:
                time.sleep(INTERVAL - elapsedTime)

def main():
    global client
    parser = argparse.ArgumentParser()
    for key in ARG_DEFAULTS.keys(): 
        parser.add_argument(f'--{key}', default=argparse.SUPPRESS)
    for key in FLAG_MASKS.keys():
        parser.add_argument(f'--{key}', action='store_true')
    parsed = parser.parse_args()

    update_thread = threading.Thread(target=update, daemon=True)
    update_thread.start()
    keyboard_thread = threading.Thread(target=readkey, daemon=True)
    keyboard_thread.start()

    try:
        client = init_client_node(on_receive, parsed)
        run_client_node(client)
    except KeyboardInterrupt:
        print('Interrupt')
    finally:
        terminated_event.set()
        update_thread.join()
        print('press any key to exit...')
        keyboard_thread.join()
