from enum import IntEnum, auto
import struct

class MSGTYPE(IntEnum):
    NONE = 0
    INIT_REQUEST = auto()
    INIT_RESPONSE = auto()
    START_REQUEST = auto()
    START_RESPONSE = auto()
    SEND = auto()
    RECV = auto()
    MAX = auto()

FORMATTER = {
    MSGTYPE.INIT_REQUEST: struct.Struct('!f'),
    MSGTYPE.INIT_RESPONSE: struct.Struct('!B256s'),
    MSGTYPE.START_REQUEST: struct.Struct('!f16sB'),
    MSGTYPE.START_RESPONSE: struct.Struct('!B256sf16sB'),
    MSGTYPE.SEND: struct.Struct('!ff'),
    MSGTYPE.RECV: struct.Struct('!B256si1080f3f3fBf')
}
