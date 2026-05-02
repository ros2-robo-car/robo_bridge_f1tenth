import struct

class MSGTYPE:
    REQUEST = 0
    RESPONSE = 1
    SEND = 2
    RECV = 3

class STATUS:
    IDLE = 0
    BUSY = 1
    ERROR = 0xff

sock_format = {
    MSGTYPE.REQUEST: struct.Struct('!fB16s'),
    MSGTYPE.RESPONSE: struct.Struct('!B256sfB16s'),
    MSGTYPE.SEND: struct.Struct('!ff'),
    MSGTYPE.RECV: struct.Struct('!i1080f3f3fBBf')
}