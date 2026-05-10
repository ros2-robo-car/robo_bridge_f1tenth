from collections.abc import Callable
import sys, termios

KEYCODE_UP = 0x41
KEYCODE_DOWN = 0x42
KEYCODE_RIGHT = 0x43
KEYCODE_LEFT = 0x44

class ConsoleKeyReader:
    def __init__(self):
        self.fd = sys.stdin.fileno()

    def __enter__(self):
        # tcgetattr -> [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
        print("Key Reading Activated")
        self.old = termios.tcgetattr(self.fd)
        self.new = termios.tcgetattr(self.fd)

        self.new[3] &= ~(termios.ICANON | termios.ECHO)
        self.new[6][termios.VEOL] = 1
        self.new[6][termios.VEOF] = 2
        self.new[6][termios.VTIME] = 0
        self.new[6][termios.VMIN] = 1

        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.new)
        return self

    def __exit__(self, exit_type, exit_value, exit_traceback):
        termios.tcsetattr(self.fd, termios.TCSANOW, self.old)
        print("Key Reading Exit")

    def readone(self):
        return sys.stdin.read(1)
