from enum import IntEnum, auto
import struct

class MSGTYPE(IntEnum):
    REQUEST = auto()
    RESPONSE = auto()
    SEND = auto()
    RECV = auto()
    MAX = auto()

class STATUS(IntEnum):
    READY = auto()
    RUNNING = auto()
    DONE = auto()
    BUSY = auto()
    ERROR = auto()
    MAX = auto()

FORMATTER = {
    MSGTYPE.REQUEST: struct.Struct('!fB16s'),
    MSGTYPE.RESPONSE: struct.Struct('!B256sfB16s'),
    MSGTYPE.SEND: struct.Struct('!ff'),
    MSGTYPE.RECV: struct.Struct('!i1080f3f3fBBf')
}