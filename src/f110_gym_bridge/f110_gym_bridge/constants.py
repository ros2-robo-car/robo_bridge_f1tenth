from enum import IntEnum, auto
import struct

class MSGTYPE(IntEnum):
    REQUEST = 0
    RESPONSE = auto()
    SEND = auto()
    RECV = auto()
    MAX = auto()

FORMATTER = {
    MSGTYPE.REQUEST: struct.Struct('!fB16s'),
    MSGTYPE.RESPONSE: struct.Struct('!B256sfB16s'),
    MSGTYPE.SEND: struct.Struct('!ff'),
    MSGTYPE.RECV: struct.Struct('!B256si1080f3f3fBf')
}
