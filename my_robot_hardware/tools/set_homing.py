#!/usr/bin/env python3
"""One-time homing for the JawPixARM servos (run with ROS stopped, torque off).

Makes every servo read 2048 ticks (= 0 rad in the URDF) in the URDF zero pose, by writing
its Homing Offset register (EEPROM, kept after power-off).

  python3 set_homing.py --show            print where each joint is now, change nothing
  python3 set_homing.py                   arm held in the zero pose -> write the offsets
  python3 set_homing.py --port /dev/u2d2  other serial port (default /dev/ttyUSB0)
"""
import argparse
import sys
import time

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

ADDR_HOMING_OFFSET = 20     # 4 bytes, signed, EEPROM
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR = 70
ADDR_PRESENT_POSITION = 132  # 4 bytes, signed
CENTER = 2048
TICKS_PER_DEG = 4096 / 360.0
NAMES = {1: 'joint1 base', 2: 'joint2 shoulder', 3: 'joint3 elbow', 4: 'joint4 wrist pitch',
         5: 'joint5 wrist roll', 6: 'joint6 gripper'}


def signed32(v):
    return v - (1 << 32) if v & 0x80000000 else v


def describe(hw):
    parts = [n for bit, n in ((0x01, 'Input Voltage'), (0x04, 'Overheating'), (0x10, 'Electrical Shock'),
                              (0x20, 'Overload')) if hw & bit]
    return ' + '.join(parts) or 'none'


def hardware_error(pkt, port, dxl_id):
    value, res, _ = pkt.read1ByteTxRx(port, dxl_id, ADDR_HARDWARE_ERROR)
    return value if res == COMM_SUCCESS else 0


def read4(pkt, port, dxl_id, addr):
    value, res, err = pkt.read4ByteTxRx(port, dxl_id, addr)
    # bit 7 of the error byte only says "this servo has a latched hardware error"; the data is still valid
    if res != COMM_SUCCESS or (err & 0x7F) != 0:
        raise RuntimeError(f'ID {dxl_id}: read {addr} failed ({pkt.getTxRxResult(res)} {pkt.getRxPacketError(err)})')
    return signed32(value)


def write4(pkt, port, dxl_id, addr, value):
    res, err = pkt.write4ByteTxRx(port, dxl_id, addr, value & 0xFFFFFFFF)
    if res != COMM_SUCCESS or (err & 0x7F) != 0:
        raise RuntimeError(f'ID {dxl_id}: write {addr} failed ({pkt.getTxRxResult(res)} {pkt.getRxPacketError(err)})')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=1000000)
    ap.add_argument('--ids', default='1,2,3,4,5,6')
    ap.add_argument('--show', action='store_true', help='only print the current angles')
    ap.add_argument('--yes', action='store_true', help='do not ask for confirmation')
    a = ap.parse_args()
    ids = [int(x) for x in a.ids.split(',')]

    port, pkt = PortHandler(a.port), PacketHandler(2.0)
    if not port.openPort() or not port.setBaudRate(a.baud):
        sys.exit(f'Cannot open {a.port} at {a.baud} baud (U2D2 connected to this machine? ROS stopped?)')

    def table(title):
        print(title)
        for i in ids:
            pos = read4(pkt, port, i, ADDR_PRESENT_POSITION)
            off = read4(pkt, port, i, ADDR_HOMING_OFFSET)
            deg = (pos - CENTER) / TICKS_PER_DEG
            flag = '   <-- outside +/-90 deg' if abs(deg) > 90 and i != 6 else ''
            hw = hardware_error(pkt, port, i)
            if hw:
                flag += f'   [latched error: {describe(hw)}]'
            print(f'  ID {i} {NAMES.get(i, ""):20s} {pos:6d} ticks  {deg:+7.1f} deg   homing offset {off:+6d}{flag}')

    try:
        table('Now (0 deg = the URDF zero pose):')
        if a.show:
            return
        print('\nHold the arm in the URDF zero pose (all joints at 0 in RViz):\n'
              '  base facing forward, upper arm about 23 deg forward of vertical, forearm and wrist\n'
              '  horizontal and in line, gripper level, moving jaw at 90 deg to the fixed jaw.')
        if not a.yes and input('Write the homing offsets now? [y/N] ').strip().lower() != 'y':
            print('Nothing written.')
            return
        for i in ids:
            if hardware_error(pkt, port, i):
                print(f'  ID {i}: clearing latched error ({describe(hardware_error(pkt, port, i))}) with a reboot')
                pkt.reboot(port, i)
                time.sleep(0.6)
            pkt.write1ByteTxRx(port, i, ADDR_TORQUE_ENABLE, 0)  # EEPROM is writable only with torque off
            pos = read4(pkt, port, i, ADDR_PRESENT_POSITION)
            old = read4(pkt, port, i, ADDR_HOMING_OFFSET)
            new = old + (CENTER - pos)       # present = raw + offset, so this moves present to 2048
            if abs(new) > 1044479:
                raise RuntimeError(f'ID {i}: offset {new} outside the allowed range')
            write4(pkt, port, i, ADDR_HOMING_OFFSET, new)
        table('\nAfter homing (every joint should read about 0 deg):')
    except RuntimeError as e:
        sys.exit(str(e))
    finally:
        port.closePort()


if __name__ == '__main__':
    main()
