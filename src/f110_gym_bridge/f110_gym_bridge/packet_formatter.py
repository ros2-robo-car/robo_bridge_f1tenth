from .constants import *

_type_parser = struct.Struct('!B')

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
            egoidx, scans, poses, vels, iscol, status, elapsed_time = FORMATTER[MSGTYPE.RECV].unpack(data[1:])
            obs = {}
            obs['ego_idx'], obs['scans'] = egoidx, scans
            obs['poses_x'], obs['poses_y'], obs['poses_theta'] = poses[0], poses[1], poses[2]
            obs['linear_vels_x'], obs['linear_vels_y'], obs['ang_vels_z'] = vels[0], vels[1], vels[2]
            obs['collisions'] = iscol
            attr['obs'] = obs
            attr['status'] = status
            attr['elapsed_time'] = elapsed_time
        else: 
            raise Exception(f"Invalid type {type}")
    except Exception as e:
        raise Exception(f"Exception on unpacking: {e}")
    return type, attr
