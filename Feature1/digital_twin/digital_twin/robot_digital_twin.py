import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
import serial
from math import sin, cos, pi
import threading

class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)               

class RobotDigitalTwin(Node):
    #the location of the robot, x, y and rotation
    x=0.0
    y=0.0
    z=0.5
    angle = 0.0 #in degrees
    th=0.0 #angle in radians

    def listener_callback(self, msg):
        command = msg.data[0:5] #the actual command string
        value = int(msg.data[6:11]) #the data
        print(command)
        print(value)

        if(command == "MOVEF"):
            if(value == 600):
                print("robot moving forward")
                vx = 0.5 #velocity of x, how much we move
                # compute odometry
                delta_x = vx * cos(self.th)
                delta_y = vx * sin(self.th)
                delta_th = 0 #no change in angle
                #add the changes to the values
                self.x += delta_x
                self.y += delta_y
                self.th += delta_th
                
        elif(command == "FASTO"):
            if(value == 600):
                print("robot moving fast")
                vx = 1 #velocity of x, how much we move
                # compute odometry
                delta_x = vx * cos(self.th)
                delta_y = vx * sin(self.th)
                delta_th = 0 #no change in angle
                #add the changes to the values
                self.x += delta_x
                self.y += delta_y
                self.th += delta_th
              
        elif(command == "FASTT"):
            if(value == 600):
                print("robot moving fastest")
                vx = 1.5 #velocity of x, how much we move
                # compute odometry
                delta_x = vx * cos(self.th)
                delta_y = vx * sin(self.th)
                delta_th = 0 #no change in angle
                #add the changes to the values
                self.x += delta_x
                self.y += delta_y
                self.th += delta_th     
               
        elif(command == "MOVEB"):
            if(value == 600):
                print("robot moving backwards")
                vx = 1 #velocity of x, how much we move
                # compute odometry
                delta_x = vx * cos(self.th)
                delta_y = vx * sin(self.th)
                delta_th = 0 #no change in angle
                #add the changes to the values
                self.x -= delta_x
                self.y -= delta_y
                self.th -= delta_th

        #Obey the right-hand law. Orientation is counter-clockwise.
        elif(command == "TURNR"):
            print("robot turning right")
            if(value == 1000):
                self.angle = (self.angle - 10) #if the result is +ve
                if (self.angle < 0): self.angle = 360 - abs(self.angle) #if the result is -ve
                self.th = math.radians(self.angle) #convert to radians

        elif(command == "TURNL"):
            print("robot turning left")
            if(value == 1000):
                self.angle = ((10 + self.angle) % 360 ) #add to the angle, keeping mod 360
                self.th = math.radians(self.angle) #convert to radians

        print(self.x)
        print(self.y)
        print(self.z)
        print(self.angle)
        print(self.th)

    def __init__(self):
        super().__init__('robot_digital_twin')
        self.subscription = self.create_subscription(
            String,
            '/robot/control',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        
        
        super().__init__('robot_wall')
        self.subscription_wall = self.create_subscription(
            String,
            '/robot/control',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publish_thread = threading.Thread(target=self._publish_thread)
        self.publish_thread.daemon = True
        self.publish_thread.start()
        
    def _publish_thread(self):
        self.publisher_front = self.create_publisher(String, '/robot/front', 10)
      
        ser = serial.Serial(
            port='/dev/ttyACM0',\
            baudrate=9600,\
            parity=serial.PARITY_NONE,\
            stopbits=serial.STOPBITS_ONE,\
            bytesize=serial.EIGHTBITS,\
            timeout=0)
        
        rl = ReadLine(ser)
        
        #publish the pose regularly
        while(True):
            
            #This will publish the robot position and update regularly
            self.publisher = self.create_publisher(Twist, '/robot/pose', 10)
            msg = Twist()
            msg.linear.x = self.x
            msg.linear.y = self.y
            msg.linear.z = self.z #for height
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = self.th
            self.publisher.publish(msg)
            self.get_logger().info("Publishing to /robot/pose")
            time.sleep(0.1) #2/5 of a second
            
            
            line = rl.readline().decode()
            clean = line[1:] #removes the [
            clean2 = clean[:-2] #removes the ]\n
            readings = clean2.split(',')
            readings = readings[0]
            wall_distance = String()  
            wall_distance.data = readings.replace('cm','')
            self.publisher_front.publish(wall_distance)
            self.get_logger().info('Publishing: "%s"' % wall_distance.data)
            
            #TODO
            #PUBLISH THE WALL POSE REGULARLY COMPARED TO ROBOT
            
            self.publisher = self.create_publisher(Twist, '/robot/wall', 10)
            msg2 = Twist()
            msg2.linear.x = float(wall_distance.data)
            msg2.linear.y = self.y
            msg2.linear.z = self.z #for height
            msg2.angular.x = 0.0
            msg2.angular.y = 0.0
            msg2.angular.z = self.th
            self.publisher.publish(msg2)
            self.get_logger().info("Publishing to /robot/wall")
            
            print(msg2)
            
            
            
def main(args=None):
    rclpy.init(args=args)

    robot = RobotDigitalTwin()
    rclpy.spin(robot)

    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
