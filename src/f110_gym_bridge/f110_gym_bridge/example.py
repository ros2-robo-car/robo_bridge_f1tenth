import argparse, keyboard, threading, time
from f110_gym_bridge_interface import *
from f110_gym_bridge.client import *

key_state_lock = threading.Lock()
key_state = {
    "up": False,
    "down": False,
    "right": False,
    "left": False,
}
terminated_event = threading.Event()
INTERVAL = 1/40


def key_event_callback(e):
    key_state_lock.acquire()
    key_state[e.name] = True if e.event_type == "down" else False
    key_state_lock.release()

def on_receive(msg: Recv):
    pass

client = F110GymClient(on_receive)

def update():
    steer, speed = 0., 0.
    quit = False
    while not terminated_event.is_set():
        curTime = time.time()
        key_state_lock.acquire()
        speed = (key_state["up"]) * 5.0
        steer = (key_state["left"] - key_state["right"]) * 1.0
        key_state_lock.release()

        act = Act()
        act.steer = steer
        act.speed = speed
        client.send(act)

        elapsedTime = time.time() - curTime
        if elapsedTime < INTERVAL:
            time.sleep(INTERVAL - elapsedTime)

def main():
    parser = argparse.ArgumentParser()
    for key in ARG_DEFAULTS.keys(): 
        parser.add_argument(f'--{key}', default=argparse.SUPPRESS)
    for key in FLAG_MASKS.keys():
        parser.add_argument(f'--{key}', action='store_true')
    parsed = parser.parse_args()

    keyboard.hook(key_event_callback)
    t = threading.Thread(target=update, daemon=True)
    t.start()

    try:
        run_client_node(parsed)
    except KeyboardInterrupt:
        pass
    finally:
        terminated_event.set()