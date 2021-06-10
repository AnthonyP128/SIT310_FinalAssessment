import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ScaredRobot(Node):

    def listener_callback(self, msg):
        print(msg.data)
        front = int(msg.data[:-2]) #remove the 'cm' and convert to int.
        if front < 10:
            msg = String()
            msg.data = "MOVEB:0100\n"
            self.publisher.publish(msg)


    def __init__(self):
        super().__init__('ScaredRobot')
        self.publisher = self.create_publisher(String, '/robot/control', 10)

        self.subscription = self.create_subscription(
            String,
            '/robot/front',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

def main(args=None):
    rclpy.init(args=args)

    scaredrobot = ScaredRobot()
    rclpy.spin(scaredrobot)
    
    scaredrobot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
