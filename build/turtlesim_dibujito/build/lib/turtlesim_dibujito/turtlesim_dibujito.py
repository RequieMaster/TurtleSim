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
        self.desired_x2 = 0.0  
        self.desired_y2 = 0.0  
        self.err_threshhold = 0.1
        self.step = 0

        # Publishers and Subscriber
        self.my_pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.my_pose_sub2 = self.create_subscription(Pose, "/turtle/pose", self.pose_callback2, 10)
        
        #http://wiki.ros.org/turtlesim#Published_Topics
        
        	#Velocity
        self.my_vel_command = self.create_publisher(Twist, "/turtle1/cmd_vel", 10) 
        self.my_vel_command2 = self.create_publisher(Twist, "/turtle/cmd_vel", 10) 
      
     ###TODO Servicion no se pueden usar para hacer publishers, usar mensajes custom
        	#Color
        self.cliColor = self.create_client(SetPen, '/turtle1/set_pen')
        self.reqColor = SetPen.Request()
        self.cliColor2 = self.create_client(SetPen, '/turtle/set_pen')
        self.reqColor2 = SetPen.Request()
        
        	#Teleport
        self.cliTp = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.reqTp = TeleportAbsolute.Request()
        self.cliTp2 = self.create_client(TeleportAbsolute, '/turtle/teleport_absolute')
        self.reqTp2 = TeleportAbsolute.Request()
        
        	#Spawn
        self.cliSpawn = self.create_client(Spawn, 'spawn')
        self.reqSpawn = Spawn.Request()
        
            #Kill
        self.cliKill = self.create_client(Kill, 'kill')
        self.reqKill = Kill.Request()
        

    def send_request_Color(self,r, g, b, width, off, num_turtle):
        self.get_logger().info("req Color")
        if(num_turtle == 1):
            self.reqColor.r = r
            self.reqColor.b = b
            self.reqColor.g = g
            self.reqColor.width = width
            self.reqColor.off = off
            future = self.cliColor.call_async(self.reqColor)
        else:
            self.reqColor2.r = r
            self.reqColor2.b = b
            self.reqColor2.g = g
            self.reqColor2.width = width
            self.reqColor2.off = off
            future = self.cliColor2.call_async(self.reqColor2)
        #rclpy.spin_until_future_complete(self, future)
        return future.result()
        
    def send_request_Teleport(self, x, y, theta, num_turtle):
        self.get_logger().info("req TP")
        if(num_turtle == 1):
            self.reqTp.x = x
            self.reqTp.y = y
            self.reqTp.theta = theta
            future = self.cliTp.call_async(self.reqTp)
        else:
            self.reqTp2.x = x
            self.reqTp2.y = y
            self.reqTp2.theta = theta
            future = self.cliTp2.call_async(self.reqTp2)
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
        
    def send_request_Kill(self, name):
        self.get_logger().info("req Kill")
        self.reqKill.name = name
        future = self.cliKill.call_async(self.reqKill)
        #rclpy.spin_until_future_complete(self, future)
        return future.result()

        
    def update_pose(self, err_x, err_y,msg):
        self.get_logger().info(f"Current x={msg.x} current y={msg.y} and current angle = {msg.theta}")
        # Calculate errors in position
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
        #self.get_logger().info(f"Desired Angle = {desired_theta} current angle {msg.theta} Error angle {err_theta}")
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
        return l_v,a_v


    def my_velocity_cont(self, l_v, a_v, turt):
        
        #Actualiza SetPen, Teleport y Spawn
        self.step_controller()
        
        #self.get_logger().info(f"Commanding liner ={l_v} and angular ={a_v}")
        my_msg = Twist()
        my_msg.linear.x = l_v
        my_msg.angular.z = a_v
        self.get_logger().info(f"turtle {turt} velocity command publish")   
        if(turt == 1):
            self.my_vel_command.publish(my_msg)
        else:
            self.my_vel_command2.publish(my_msg)

    def pose_callback(self, msg: Pose):
        self.get_logger().info(f"turtle 1 pose callback")
        err_x = self.desired_x - msg.x
        err_y = self.desired_y - msg.y
        l_v, a_v = self.update_pose(err_x, err_y, msg)
        self.my_velocity_cont(l_v, a_v, 1)

    def pose_callback2(self, msg: Pose):
        self.get_logger().info(f"turtle 2 pose callback")   
        err_x = self.desired_x2 - msg.x
        err_y = self.desired_y2 - msg.y
        l_v, a_v = self.update_pose(err_x, err_y, msg)
        self.my_velocity_cont(l_v, a_v, 2)


    def step_controller(self):
        self.get_logger().info(f"step_controller step: {self.step}")
  
        if(self.step == 0):
            self.step += 1
        elif(self.step == 1):
            self.desired_x = 10.0  
            self.desired_y = 2.3
            self.send_request_Color(0,0,0,300,1,1)
            self.send_request_Teleport(10.0,2.3,190.0,1)
        elif(self.step == 2):
            self.desired_x = 0.0  
            self.desired_y = 2.3
            self.send_request_Color(0,255,0,100,0,1)
            self.send_request_Teleport(0.0,2.3,190.0,1)
        elif(self.step == 3):
            self.desired_x = 3.0  
            self.desired_y = 3.0
            self.send_request_Color(0,255,0,100,1,1)
            self.send_request_Spawn(6.5,3.0,70.0,"turtle")
            self.send_request_Color(0,0,0,10,1,2)
            self.send_request_Teleport(3.0,3.0,70.0,1)
        elif(self.step == 5):
            self.send_request_Color(128,68,0,10,0,2)
            self.send_request_Color(128,68,0,10,0,1)
            self.desired_x = 3.0  
            self.desired_y = 7.5
            self.desired_x2 = 6.5
            self.desired_y2 = 7.5
        elif(self.step == 6):
            self.desired_x = 4.5  
            self.desired_y = 9
            self.desired_x2 = 4.5
            self.desired_y2 = 9
        elif(self.step == 7):
            self.send_request_Color(0,0,0,10,1,1)
        elif(self.step == 8):
            self.send_request_Teleport(10.0,10.5,180.0,1)
            self.step += 1
        elif(self.step == 9):
            self.send_request_Color(255,255, 0, 100,0,1)
            self.desired_x = 10.0  
            self.desired_y = 10.5
        elif(self.step == 130):
            self.send_request_Kill("turtle")
            self.send_request_Kill("turtle1")
        

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
