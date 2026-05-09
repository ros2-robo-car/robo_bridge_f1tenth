from .constants import *

_type_parser = struct.Struct('!B')

def struct_size(type: MSGTYPE) -> int:
    return FORMATTER[type].size + 1

def pack(type: MSGTYPE, attr: dict) -> bytes:
    try:
        if type == MSGTYPE.REQUEST:
            return _type_parser.pack(type) + FORMATTER[MSGTYPE.REQUEST].pack(
                attr['timestep'], 
                attr['flags'], 
                attr['map'].encode(encoding='ascii')
            )
        elif type == MSGTYPE.SEND:
            return _type_parser.pack(type) + FORMATTER[MSGTYPE.SEND].pack(
                attr['steer'], 
                attr['speed']
            )
        else: 
            raise Exception(f"Invalid type {type}")
    except Exception as e:
        raise Exception(f"Exception on packing: {e}")

def unpack(data: bytes) -> tuple[MSGTYPE, dict]:
    try:
        type = _type_parser.unpack(data[0:1])[0]
        attr = {}
        if type == MSGTYPE.RESPONSE:
            status, msg, timestep, flags, map = FORMATTER[MSGTYPE.RESPONSE].unpack(data[1:])
            attr['status'] = status
            attr['msg'] = msg.decode(encoding='ascii').strip('\x00')
            attr['timestep'] = timestep
            attr['flags'] = flags
            attr['map'] = map.decode(encoding='ascii').strip('\x00')
        elif type == MSGTYPE.RECV:
            parsed = FORMATTER[MSGTYPE.RECV].unpack(data[1:])
            status, msg, egoidx = parsed[0], parsed[1], parsed[2]
            scans, poses, vels = parsed[3:1083], parsed[1083:1086], parsed[1086:1089]
            iscols, elapsed_time = parsed[1089], parsed[1090]
            attr['ego_idx'], attr['scans'] = egoidx, scans
            attr['poses_x'], attr['poses_y'], attr['poses_theta'] = poses[0], poses[1], poses[2]
            attr['linear_vels_x'], attr['linear_vels_y'], attr['ang_vels_z'] = vels[0], vels[1], vels[2]
            attr['collisions'] = iscols
            attr['status'] = status
            attr['msg'] = msg.decode(encoding='ascii').strip('\x00')
            attr['elapsed_time'] = elapsed_time
        else: 
            raise Exception(f"Invalid type {type}")
    except Exception as e:
        raise Exception(f"Exception on unpacking: {e}")
    return type, attr
