import rclpy
from rclpy.node import Node
from std_msgs.msg import String
#Hardware libraries
import board #maps to physical pins on the board
import busio #for I2C communication
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo


class HandControllerNode(Node):
    """
    A ROS 2 node to control a hand using PCA9685 servo controller.
    It uses the PCA9685 library to control the servos.
    It subscribes to the /hand_command topic and publishes the servo commands to the PCA9685.
    """
    def __init__(self): # No need for timer becuase subscriber callback runs whenever message arrives
        super().__init__("hand_controller")

        # Handle potential hardware errors first
        try:
            # I2C is class in busio library, board.SCL (timing signal) and board.SDA (data signal) are physical pins for I2C communication
            i2c = busio.I2C(board.SCL, board.SDA) #create I2C object
            self.pca = PCA9685(i2c)    #create PCA9685 object
            self.pca.frequency = 50 #50 Hz is the default frequency for servos, faster than this and servos are not built to process it

            '''
            # see the following is how you manually do it for your understanding
            # Define servo channels
            #self.servo_channels = [0, 1, 2, 3, 4]

            # Define servo angles
            #self.OPEN_ANGLE = 0 # Fully open
            #self.GRASP_ANGLE = 180 # Fully closed
            '''
            # Create servo objects
            self.servos = [servo.Servo(self.pca.channels[i]) for i in range(5)]


            self.get_logger().info("PCA9685 initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Error initializing PCA9685: {e}")
            raise #stop node if hardware init fails

        # Create susbscriber
        self.subscriber = self.create_subscription(String, "/hand_command", self.hand_command_callback, 10)
        self.get_logger().info('Hand Controller Node has been started.')

    # Msg comes from ROS 2 when command_publisher publishes a message. By default any msg is related to a hand_control command.
    # So, it doesn't need to know the string name or something. Rather if it is something different or confusing you would just use a different topic!
    def hand_command_callback(self, msg): 
        """
        Called when a hand command is received.
        """
        self.get_logger().info(f"Received hand command: {msg.data}")
        if msg.data == "OPEN":
            self.set_all_servos(0)
            self.get_logger().info("OPEN command received")
        elif msg.data == "GRASP":
            self.set_all_servos(180)
            self.get_logger().info("GRASP command received")
        else:
            self.get_logger().warn(f"Invalid hand command: {msg.data}")


    def set_all_servos(self, angle):
        """
        Set all servos to the same angle: 0-180 degrees
        # channels 0 and 1  and 3 (thunmb and pointer and ring) close from 0-180
        # channels 2 and 4 (middle and pinky), closes from 180-0
        """

        for i, s in enumerate(self.servos): #s is servo object, i is index
            if i == 0 or i == 1 or i == 3:  # thumb, index, ring
                servo_angle = angle  # Use original angle
            else:  # middle and pinky
                servo_angle = 180 - angle  # Invert the angle
            s.angle = servo_angle
            self.get_logger().info(f"Set servo {i} to {servo_angle} degrees")
        
        '''
        try:
            the following is how you manually do it for your understanding
            # convert angle to PWM duty cycle; basically mapping 0-180 to 0-65535
            # duty cycle is the value pulse is high (16-bit since that's how PCA9685 works). Since hardware limitation is 1-2 ms pulse width, duty cycle is 5-10% for 0-180 degrees
            
            # Linear mapping: y = mx + b
            # where m = slope, b = y-intercept
    
            # slope = (6554 - 3277) / (180 - 0) = 3277 / 180
            # y-intercept = 3277 (when angle = 0)
    
            duty_cycle = int(3277 + (angle * ( 3277) / 180)) # 6554 is the max duty cycle, 3277 is the min duty cycle

            # Safety check
            if duty_cycle < 3277 or duty_cycle > 6554:
                self.get_logger().warn(f"Duty cycle {duty_cycle} out of safe range")
                return
            
            # set all servos to the same duty cycle
            for channel in self.servo_channels:
                self.pca.channels[channel].duty_cycle = duty_cycle
                self.get_logger().info(f"Set servo {channel} to {angle} degrees")
        except Exception as e:
            self.get_logger().error(f"Error setting servos: {e}")
        '''
    

def main(args=None):
    rclpy.init(args=args)
    node = HandControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


