#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle

from math import sqrt, atan2, pi
from functools import partial


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")

        
        self.linear_kp = 2.0
        self.angular_kp = 6.0

        # Initialize 
        self.next_target_turtle = None
        self.master_x = None

        self.turtle_pose_sub_ = self.create_subscription(
            Pose, "turtle1/pose", self.callback_turtle_pose, 10)
        self.alive_turtles_sub_ = self.create_subscription(
            TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)
        self.velocity_pub_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.control_timer_ = self.create_timer(0.01, self.control_master_turtle)
        
        self.get_logger().info("Turtle Controller has been started")
        


    # Turtle Pose Subscriber Callback
    def callback_turtle_pose(self, pose):
        self.master_x = pose.x
        self.master_y = pose.y
        self.master_theta = pose.theta

    # Alive Turtles Subscriber Callback
    def callback_alive_turtles(self, alive_turtles):
        self.alive_turtles = alive_turtles.turtles
        
        if len(self.alive_turtles) > 0:
            distances = []
            for turtle in self.alive_turtles:
                dist_x = turtle.x - self.master_x
                dist_y = turtle.y - self.master_y
                dist = self.compute_distance(dist_x, dist_y)
                distances.append(dist)           
            i =  distances.index(min(distances))  
            self.next_target_turtle = self.alive_turtles[i] 



    # ------------------    CONTROL MASTER TURTLE    ------------------

    # Function to compute euclidean distance
    def compute_distance(self, dist_1, dist_2):
        dist = sqrt(dist_1**2 + dist_2**2)
        return dist

    # Control Loop Function (Velocity Publisher Callback)
    def control_master_turtle(self):
        if self.master_x == None or self.next_target_turtle  == None:
            return
        
        if len(self.alive_turtles) == 0:
            return
 
        dist_x = self.next_target_turtle.x - self.master_x
        dist_y = self.next_target_turtle.y - self.master_y
        dist = self.compute_distance(dist_x, dist_y)

        vel_msg = Twist()

        if dist > 0.5:
            # position velocities
            vel_msg.linear.x = self.linear_kp * dist
            # vel_msg.linear.y = self.linear_kp * dist_y

            # angular velocity
            goal_theta_ = atan2(dist_y, dist_x)
            theta_diff = goal_theta_ - self.master_theta
            if theta_diff > pi:
                theta_diff -= 2 * pi
            elif theta_diff < -pi:
                theta_diff += 2 * pi
            vel_msg.angular.z = self.angular_kp * theta_diff

        else:
            # Target is reached
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.angular.z = 0.0
            
            # Catch Turtle
            caught_turtle = self.next_target_turtle.name
            self.catch_turtle(caught_turtle)
            self.next_target_turtle = None
            
        self.velocity_pub_.publish(vel_msg)
        

    # # ------------------    CATCH TURTLE    ------------------
    
    # Catch Turtle Client
    def catch_turtle (self, turtle_name):
        catch_client_ = self.create_client(CatchTurtle, "catch_turtle")
        while not catch_client_.wait_for_service(1.0):
            self.get_logger().info("Waiting for the turtlesim_node catch_turtle server...")

        request = CatchTurtle.Request()
        request.turtle_name = turtle_name

        future = catch_client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_kill_service, turtle_name=turtle_name))

    # Kill Turtle Client Callback
    def callback_call_kill_service(self, future, turtle_name):
        try:
            response = future.result()
            self.get_logger().info("%s caught successfully" %turtle_name)
        except Exception as e:
            self.get_logger().info("Service call failed %r" % (e,))

    # # ----------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
