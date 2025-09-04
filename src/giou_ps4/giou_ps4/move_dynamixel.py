import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from dynamixel_sdk import *
import math

# ====== DYNAMIXEL PARAMETERS FOR XL430-W250-T ======
DEVICENAME = "/dev/ttyUSB0"
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0

# Control table addresses
ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

TICKS_PER_REV = 4096  # XL430 resolution

class DynamixelBridge(Node):
    def __init__(self):
        super().__init__('dynamixel_bridge')

        self.servo_ids = list(range(1, 7))
        self.goal_positions = {}
        self.changed = set()

        # Setup handlers
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        if not self.portHandler.openPort():
            self.get_logger().error("❌ Failed to open port.")
            return
        if not self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().error("❌ Failed to set baudrate.")
            return

        # Disable torque + set position mode
        for dxl_id in self.servo_ids:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_OPERATING_MODE, 3)

        # Publishers for feedback (radians)
        self.joint_publishers = {
            i: self.create_publisher(Float64, f'/dynamixel/joint{i}_state', 10)
            for i in self.servo_ids
        }

        # Read actual present positions and set as goals
        self.goal_positions = self.read_all_present_positions()

        # Enable torque simultaneously
        self.enable_all_torque()

        # Subscribers for teleop commands
        for i in self.servo_ids:
            self.create_subscription(
                Float64,
                f'/dynamixel/joint{i}_position_controller/command',
                lambda msg, id=i: self.store_position(id, msg.data),
                10
            )

        # Timers
        self.timer_cmd = self.create_timer(0.05, self.send_all_positions)
        self.timer_feedback = self.create_timer(0.1, self.publish_all_positions)

        self.get_logger().info("✅ Dynamixel Bridge ready for XL430-W250-T")

    def ticks_to_rad(self, ticks):
        return (ticks / TICKS_PER_REV) * 2 * math.pi

    def rad_to_ticks(self, radians):
        radians = radians % (2 * math.pi)  # wrap around
        return int((radians / (2 * math.pi)) * TICKS_PER_REV)

    def read_all_present_positions(self):
        positions = {}
        for dxl_id in self.servo_ids:
            pos, result, error = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, ADDR_PRESENT_POSITION)
            if result != COMM_SUCCESS:
                self.get_logger().error(f"[ID {dxl_id}] Read error: {self.packetHandler.getTxRxResult(result)}")
                pos = 2048
            elif error != 0:
                self.get_logger().error(f"[ID {dxl_id}] Packet error: {self.packetHandler.getRxPacketError(error)}")
                pos = 2048
            positions[dxl_id] = pos
            rad = self.ticks_to_rad(pos)
            self.get_logger().info(f"[ID {dxl_id}] Initial position: {pos} ticks ({rad:.2f} rad)")
            # Publish initial state
            msg = Float64()
            msg.data = rad
            self.joint_publishers[dxl_id].publish(msg)
        return positions

    def store_position(self, dxl_id, position_rad):
        ticks = self.rad_to_ticks(position_rad)
        ticks = max(0, min(4095, ticks))
        self.goal_positions[dxl_id] = ticks
        self.changed.add(dxl_id)

    def send_all_positions(self):
        if not self.changed:
            return

        groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, 4)
        for dxl_id in self.changed:
            param = [
                self.goal_positions[dxl_id] & 0xFF,
                (self.goal_positions[dxl_id] >> 8) & 0xFF,
                (self.goal_positions[dxl_id] >> 16) & 0xFF,
                (self.goal_positions[dxl_id] >> 24) & 0xFF,
            ]
            groupSyncWrite.addParam(dxl_id, bytearray(param))

        result = groupSyncWrite.txPacket()
        if result != COMM_SUCCESS:
            self.get_logger().error(f"Failed to send group position: {self.packetHandler.getTxRxResult(result)}")
        else:
            self.get_logger().info("🔄 Group position sent")
        groupSyncWrite.clearParam()
        self.changed.clear()

    def publish_all_positions(self):
        """Publish feedback in radians for each joint"""
        for dxl_id in self.servo_ids:
            pos, result, error = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, ADDR_PRESENT_POSITION)
            if result == COMM_SUCCESS and error == 0:
                rad = self.ticks_to_rad(pos)
                msg = Float64()
                msg.data = rad
                self.joint_publishers[dxl_id].publish(msg)

    def enable_all_torque(self):
        groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_TORQUE_ENABLE, 1)
        for dxl_id in self.servo_ids:
            groupSyncWrite.addParam(dxl_id, bytearray([TORQUE_ENABLE]))
        result = groupSyncWrite.txPacket()
        if result == COMM_SUCCESS:
            self.get_logger().info("⚡ Torque enabled on all servos")
        else:
            self.get_logger().error(f"Failed to enable torque: {self.packetHandler.getTxRxResult(result)}")
        groupSyncWrite.clearParam()

    def disable_all_torque(self):
        groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_TORQUE_ENABLE, 1)
        for dxl_id in self.servo_ids:
            groupSyncWrite.addParam(dxl_id, bytearray([TORQUE_DISABLE]))
        result = groupSyncWrite.txPacket()
        if result == COMM_SUCCESS:
            self.get_logger().info("🛑 Torque disabled on all servos")
        else:
            self.get_logger().error(f"Failed to disable torque: {self.packetHandler.getTxRxResult(result)}")
        groupSyncWrite.clearParam()

    def destroy_node(self):
        self.disable_all_torque()
        self.portHandler.closePort()
        super().destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    node = DynamixelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
