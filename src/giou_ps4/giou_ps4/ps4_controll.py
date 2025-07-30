import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from rclpy.publisher import Publisher

class PS4DynamixelTeleop(Node):
    def __init__(self):
        super().__init__('ps4_dynamixel_teleop')

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
        self.increment = 0.08 #multiply factor for movement speed
    
        self.last_buttons = [] #track previous button state
        self.get_logger().info("Node initialized. Waiting for joystick input...")

    def joy_callback(self, msg: Joy):

        if not self.last_buttons:
            self.last_buttons = msg.buttons
            return

        
        L1_pressed = msg.buttons[4] == 1 and self.last_buttons[4] == 0 #L1 button
        R1_pressed = msg.buttons[5] == 1 and self.last_buttons[5] == 0 #R1 button

        #Commute between joints with R1 (button [5]) and L1 (button [4])
      
        if R1_pressed and self.selected_joint < 6:
            self.selected_joint += 1
            self.get_logger().info(f'Selected joint {self.selected_joint}')
        elif L1_pressed and self.selected_joint > 1:
            self.selected_joint -= 1
            self.get_logger().info(f'Selected joint {self.selected_joint}')

        #angle limits
        MAX_ANGLE = math.pi 
        MIN_ANGLE = -math.pi

        self.axis_value = 0.0
        self.axis_threshold = 0.5 #stick must be pushed beyond this to trigger

        #Use the right stick vertical (axis 3 and 4) 
        axis_value = msg.axes[4] #adjust if needed
        
        #detect positive edge (increment)
        if axis_value > self.axis_threshold and self.last_axis_value <= self.axis_threshold:
            self.current_positions[self.selected_joint] += self.increment
            
        #detect negative edge (decrement)
        elif axis_value < -self.axis_threshold and self.last_axis_value >= -self.axis_threshold:
            self.current_positions[self.selected_joint] -= self.increment
        
        #clamp if needed comment/uncomment the line bellow
        self.current_positions[self.selected_joint] = max(MIN_ANGLE, min(MAX_ANGLE, self.current_positions[self.selected_joint]))
        cmd = Float64()
        cmd.data = self.current_positions[self.selected_joint]
        self.joint_publishers[self.selected_joint].publish(cmd)
        self.get_logger().info (
        f'Joint {self.selected_joint} -> {cmd.data:.2f} rad'
        )

        # Update last axis value
        self.last_axis_value = axis_value
        # Update last button state
        self.last_buttons = msg.buttons

def main(args=None):
    rclpy.init(args=args)
    node = PS4DynamixelTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()   

if __name__ == '__main__':
    main()