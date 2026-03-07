import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial 
import time

class SerialBridgeNode(Node):

    def __init__(self):
        super().__init__('serial_bridge_node')
        
        # connect arduino
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 1)
            time.sleep(2)
            self.get_logger().info('Serial connection established on /dev/ttyACM0')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            return
        
        # subscribe to cmd_vel topic from laptop
        
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
             10
        )
        
        self.get_logger().info('Serial Bridge Node Started - Listening to /cmd_vel')
        
    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        
        left_speed = int((linear - angular) * 255)
 #       right_speed = int((linear + angular) * 255)
        
        left_speed = max(-255, min(255, left_speed))
 #        right_speed = max(-255, min(255, right_speed))

        # send comment to arduino
  #        cmd = f"L{left_speed}, R{right_speed}\n"
        cmd = f"L{left_speed}\n"
        self.ser.write(cmd.encode())

        self.get_logger().info(f'Sent to Arduino: {cmd.strip()}')
    
    def __del__(self):
        if hasattr(self, 'ser'):
            self.ser.close()

def main(args=None):   
    rclpy.init(args=args)
    node = SerialBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()     
