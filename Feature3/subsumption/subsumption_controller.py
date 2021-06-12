import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard
import getpass
import time

class KeyboardController(Node):
    def on_release(self,key):
        msg = String()

        if key == keyboard.Key.up:
            print("Up")
            msg.data = "CONTF"
            self.publisher.publish(msg)
        if key == keyboard.Key.down:
            print("Down")
            msg.data = "MOVEB:600"
            self.publisher.publish(msg)
        if key == keyboard.Key.left:
            print("Left")
            msg.data = "TURNL:1000"
            self.publisher.publish(msg)
        if key == keyboard.Key.right:    
            print("Right")
            msg.data = "TURNR:1000"
            self.publisher.publish(msg)
        if key == keyboard.KeyCode(char='q'):
        	print("Faster")
        	msg.data = "FASTO:600"
        	self.publisher.publish(msg)
        if key == keyboard.KeyCode(char='w'):
        	print("Fastest")
        	msg.data = "FASTT:600"
        	self.publisher.publish(msg)	
        if key == keyboard.Key.esc:
            return False
            
    def __init__(self):
        super().__init__('KeyboardController')
        self.publisher = self.create_publisher(String, '/robot/control', 10)

        # Collect events until released
        #with keyboard.Listener(
        #    on_release=self.on_release) as listener:
        #        listener.join()
        #listener.start()
        
        self.subscription = self.create_subscription(
            String,
            'robot/front',
            self.listener_callback_front_sensor,
            10)
        self.subscription
        
        self.subscription = self.create_subscription(
            String,
            'robot/left'
            self.listener_callback_left_sensor,
            10)
        self.subscription
        
        self.subscription = self.create_subscription(
            String,
            'robot/right'
            self.listener_callback_right_sensor,
            10)
        self.subscription
        
    #When detects wall in front it backs away
    def listener_callback_front_sensor(self, msg):
        front = int(msg.data[:-2])
        msg = String()
        if front > 10:
            msg.data = "MOVEB:600"
            self.publisher.publish(msg)
            print("Wall Detected")
       
    #When it detects wall from left sensor turns away from wall        
    def listener_callback_left_sensor(self, msg):
       left = int(msg.data[:-2])
       msg = String()
       if left > 15:
           msg.data = "AVOIL:1000"
           self.publisher.publish(msg)
           print("Wall Detected")
           
    #When it detects wall from right sensor turns away from wall   
    def listener_callback_right_sensor(self, msg):
        right = int(msg.data[:-2])
        msg = String()
        if right > 15
            msg.data = "AVOIR:1000"
            self.publisher.publish(msg)
            print("Wall Detected")        
            
def main(args=None):
    rclpy.init(args=args)
    controller = KeyboardController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
