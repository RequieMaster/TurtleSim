#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import Spawn
from turtlesim.srv import Kill

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtlesim_dibujito')
        self.get_logger().info("Node: Turtle Controller Started")
        
        self.turtle_name = "turtle"
        self.turtle_counter = 1
        

        self.desired_x = 9.0  # TODO Adjust as needed
        self.desired_y = 9.0  # TODO Adjust as needed

        # Publishers and Subscriber
        self.my_pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        
        #http://wiki.ros.org/turtlesim#Published_Topics
        
        	#Velocity
        self.my_vel_command = self.create_publisher(Twist, "/turtle1/cmd_vel", 10) 
      
     ###TODO Servicion no se pueden usar para hacer publishers, usar mensajes custom
        	#Color
        self.my_color_command = self.create_publisher(SetPen, "/turtle1/set_pen", 10)
        	#Teleport
        self.my_tp_command = self.create_publisher(TeleportAbsolute, "/turtle1/teleport_absolute", 10)
        	#Spawn
        self.my_spawn_command = self.create_publisher(Spawn, "/turtle1/spawn", 10)


    def pose_callback(self, msg: Pose):
        self.get_logger().info(f"Current x={msg.x} current y={msg.y} and current angle = {msg.theta}")
        # Calculate errors in position
        err_x = self.desired_x - msg.x
        err_y = self.desired_y - msg.y
        err_dist = (err_x**2+err_y**2)**0.5
        
        # Distance error (magnitude of the error vector)
        
        self.get_logger().info(f"Error in x {err_x} and error in y {err_y}")

        # Desired heading based on the position error
        desired_theta = math.atan2(err_y, err_x)
        
        # Error in heading
        err_theta = desired_theta - msg.theta
       
        # Handle wrap-around issues (e.g., if error jumps from +pi to -pi)
        while err_theta > math.pi:
            err_theta -= 2.0 * math.pi
        while err_theta < -math.pi:
            err_theta += 2.0 * math.pi
        self.get_logger().info(f"Desired Angle = {desired_theta} current angle {msg.theta} Error angle {err_theta}")
        # P (ID not required) for linear velocity (distance control)

        Kp_dist = 0.4
            


        # P (ID not required) constants for angular velocity (heading control)
        Kp_theta = 2
        

        # TODO: Add integral and derivative calculations for complete PID

        # PID control for linear velocity
        #l_v = Kp_dist * abs(err_x) # + Ki_dist * integral_dist + Kd_dist * derivative_dist
        l_v = Kp_dist * abs(err_dist) # + Ki_dist * integral_dist + Kd_dist * derivative_dist


        # PID control for angular velocity
        a_v = Kp_theta * err_theta  

        # Send the velocities
        self.my_velocity_cont(l_v, a_v)


    def my_velocity_cont(self, l_v, a_v):
        self.get_logger().info(f"Commanding liner ={l_v} and angular ={a_v}")
        my_msg = Twist()
        my_msg.linear.x = l_v
        my_msg.angular.z = a_v
        self.my_vel_command.publish(my_msg)


    def my_color_cont(self, r,g,b,width):
        #self.get_logger().info(f"Previous Color ={sense_color} and New Color ={new_color}") #TODO
        my_msg = SetPen #TODO que tipo de mensaje?
        my_msg.r = r
        my_msg.g = g
        my_msg.b = b
        my_msg.width = width
        my_msg.off = 0
        self.my_color_command.publish(my_msg)
        
        
    def my_tp_cont(self, x,y,theta):
        self.get_logger().info(f"Turtle teleported to ={x,y}") #TODO
        my_msg = TeleportAbsolute #TODO que tipo de mensaje?
        my_msg.x = x
        my_msg.y = y
        my_msg.theta = theta
        self.my_tp_command.publish(my_msg)
        
        
    def my_spawn_cont(self, x,y,theta,name):
        self.get_logger().info(f"Turtle spawned at ={x,y}") #TODO
        my_msg = Spawn #TODO que tipo de mensaje?
        my_msg.x = x
        my_msg.y = y
        my_msg.theta = theta
        my_msg.name = name
        self.my_spawn_command.publish(my_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
