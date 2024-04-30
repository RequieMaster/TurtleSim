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
        

        self.desired_x = 0.0  
        self.desired_y = 0.0  
        self.err_threshhold = 0.1
        self.wait_count = 300
        self.step = 0

        # Publishers and Subscriber
        self.my_pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        
        #http://wiki.ros.org/turtlesim#Published_Topics
        
        	#Velocity
        self.my_vel_command = self.create_publisher(Twist, "/turtle1/cmd_vel", 10) 
      
     ###TODO Servicion no se pueden usar para hacer publishers, usar mensajes custom
        	#Color
        self.cliColor = self.create_client(SetPen, '/turtle1/set_pen')
        self.reqColor = SetPen.Request()
        
        	#Teleport
        self.cliTp = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.reqTp = TeleportAbsolute.Request()
        
        	#Spawn
        self.cliSpawn = self.create_client(Spawn, '/turtle1/spawn')
        self.reqSpawn = Spawn.Request()
        

    def send_request_Color(self,r, g, b, width, off):
        self.get_logger().info("req Color")
        self.reqColor.r = r
        self.reqColor.b = b
        self.reqColor.g = g
        self.reqColor.width = width
        self.reqColor.off = off
        future = self.cliColor.call_async(self.reqColor)
        #rclpy.spin_until_future_complete(self, future)
        return future.result()
        
    def send_request_Teleport(self, x, y, theta):
        self.get_logger().info("req TP")
        self.reqTp.x = x
        self.reqTp.y = y
        self.reqTp.theta = theta
        future = self.cliTp.call_async(self.reqTp)
        #rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def send_request_Spawn(self, x, y, theta, name):
        self.get_logger().info("req Spawn")
        self.reqSpawn.x = x
        self.reqSpawn.y = y
        self.reqSpawn.theta = theta
        self.reqSpawn.name = name
        future = self.cliSpawn.call_async(self.reqSpawn)
        #rclpy.spin_until_future_complete(self, future)
        return future.result()  

    def pose_callback(self, msg: Pose):
        self.get_logger().info(f"Current x={msg.x} current y={msg.y} and current angle = {msg.theta}")
        # Calculate errors in position
        err_x = self.desired_x - msg.x
        err_y = self.desired_y - msg.y
        err_dist = (err_x**2+err_y**2)**0.5
        
        if(err_dist < self.err_threshhold):
            self.step += 1
        
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
        self.get_logger().info("vel controller")
        
        #Actualiza SetPen, Teleport y Spawn
        self.step_controller()
        
        #self.get_logger().info(f"Commanding liner ={l_v} and angular ={a_v}")
        my_msg = Twist()
        my_msg.linear.x = l_v
        my_msg.angular.z = a_v
        self.my_vel_command.publish(my_msg)
		


    def step_controller(self):
        self.get_logger().info(f"step_controller step: {self.step}")
  
        if(self.step == 0):
            #self.desired_x = 0.0  
            #self.desired_y = 1.5
            self.step += 1
        elif(self.step == 1):
            self.send_request_Color(0,0,0,100,1)
            self.send_request_Teleport(10.0,1.3,190.0)
            self.step += 1
        elif(self.step == 2):
            self.send_request_Color(0,255,0,100,0)
            self.send_request_Teleport(0.0,1.3,190.0)
            #self.send_request_Spawn(100,0,0,"turtle2")
            self.step += 1
        elif(self.step == 3):
            self.send_request_Color(0,0,0,100,1)
            self.send_request_Teleport(10.0,10.5,180.0)
            self.step += 1
        elif(self.step == 4):
            self.send_request_Teleport(10.0,10.5,180.0)
            self.desired_x = 10.0  
            self.desired_y = 10.5
            self.send_request_Color(255,255, 0, 100,0)
        

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
