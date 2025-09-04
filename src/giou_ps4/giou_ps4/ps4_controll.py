import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from rclpy.publisher import Publisher

class PS4DynamixelTeleop(Node):
    def __init__(self):

        super().__init__('ps4_dynamixel_teleop')

        # Debug startup logs
        self.get_logger().info("Turning on the PS4 controller...")
        self.get_logger().info("Waiting for the joy_node to start...")

        # Flags
        self.joy_node_detected = False
        self.joy_input_detected = False

        #subscribe to joy node, we have to run "ros2 run joy joy_node" first, then this script
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)


        #create publishers for each servo joint
        self.servo_ids = list(range(1, 7))
        self.joint_publishers = {
            i: self.create_publisher(Float64, f'/dynamixel/joint{i}_position_controller/command', 10)
            for i in self.servo_ids
        }

        self.selected_joint = 1
        self.current_positions = {i: 0.0 for i in self.servo_ids} #start at 0 radians
        self.increment = 0.08 #multiply factor for rotation speed
        self.last_buttons = [] #track previous button state
        self.axis_value = 0.0
        self.axis_threshold = 0.5 #stick must be pushed beyond this to trigger
        self.axis_gain = 0.02   # radians per callback per full stick deflection
        self.last_axis_value = 0.0

    def joy_callback(self, msg: Joy):
        if not self.joy_node_detected:
            self.joy_node_detected = True
            self.get_logger().info("joy_node started successfully!")

        if not self.joy_input_detected and (any(msg.buttons) or any(abs(a) > 0.1 for a in msg.axes)):
            self.joy_input_detected = True
            self.get_logger().info("Joystick input detected.")

        if not self.last_buttons:
            self.last_buttons = msg.buttons
            return

        # Handle joint switching
        L1_pressed = msg.buttons[4] == 1 and self.last_buttons[4] == 0
        R1_pressed = msg.buttons[5] == 1 and self.last_buttons[5] == 0

        if R1_pressed and self.selected_joint < 6:
            self.selected_joint += 1
            self.get_logger().info(f'Selected joint {self.selected_joint}')
        elif L1_pressed and self.selected_joint > 1:
            self.selected_joint -= 1
            self.get_logger().info(f'Selected joint {self.selected_joint}')

        # --- Stick control in step mode ---
        axis_value = msg.axes[4]  # right stick vertical

        threshold = 0.6  # how far you must push to register a step
        step_size = 0.1  # radians per step

        # detect stick push up
        if axis_value > threshold and self.last_axis_value <= threshold:
            self.current_positions[self.selected_joint] += step_size

        # detect stick push down
        elif axis_value < -threshold and self.last_axis_value >= -threshold:
            self.current_positions[self.selected_joint] -= step_size

        # clamp between [0, π]
        self.current_positions[self.selected_joint] = max(0.0, min(math.pi, self.current_positions[self.selected_joint]))

        # publish only if updated
        cmd = Float64()
        cmd.data = self.current_positions[self.selected_joint]
        self.joint_publishers[self.selected_joint].publish(cmd)
        self.get_logger().info(f'Joint {self.selected_joint} -> {cmd.data:.2f} rad')

        # update states
        self.last_axis_value = axis_value
        self.last_buttons = msg.buttons

def main(args=None):
    rclpy.init(args=args)
    node = PS4DynamixelTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()   

if __name__ == '__main__':
    main()