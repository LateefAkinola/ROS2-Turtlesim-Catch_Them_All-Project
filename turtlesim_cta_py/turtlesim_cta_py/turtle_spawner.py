from math import pi
import random
from functools import partial

import rclpy
from rclpy.node import Node

from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from turtlesim_cta_interfaces.msg import Turtle
from turtlesim_cta_interfaces.msg import TurtleArray
from turtlesim_cta_interfaces.srv import CatchTurtle

 
 
class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.declare_parameter("spawning_frequency", 0.5)
        self.declare_parameter("turtle_name_prefix", "Latino")
        
        self.turle_name_ = self.get_parameter("turtle_name_prefix").value
        self.spawning_feq_ = self.get_parameter("spawning_frequency").value
        
        self.alive_turtles = []
        self.turtle_counter_ = 1
        
        self.spawn_timer_ = self.create_timer(1.2, self.call_spawn_turtle)
        self.alive_turtle_pub_ = self.create_publisher(TurtleArray, "alive_turtles", 10)
        self.alive_turtle_pub_timer_ = self.create_timer(1.0 / self.spawning_feq_, self.publish_alive_turtles)
        self.catch_turtle_service = self.create_service(CatchTurtle, "catch_turtle", self.callback_catch_turtle)
        
        self.get_logger().info("Turtle Spawner has been started")
        
        
        
    # ------------------    CATCH TURTLE SERVICE   ------------------   
        
    # Catch Turtle Service Callback   
    def callback_catch_turtle(self, request, response):
        turtle_name = request.turtle_name
        
        # Kill Caught Turtle
        self.kill_turtle(turtle_name)
        response.success = True
    
        return response
    
    
    # ------------------    ALIVE TURTLE PUBLISHER  ------------------
        
    # Alive Turtles Publisher Callback 
    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles
        self.alive_turtle_pub_.publish(msg)
        
    
    # # ------------------    KILL TURTLE    ------------------
    
    # Kill Turtle Client
    def kill_turtle (self, turtle_name):
        kill_client_ = self.create_client(Kill, "kill")
        while not kill_client_.wait_for_service(1.0):
            self.get_logger().info("Waiting for the turtlesim_node kill server...")

        request = Kill.Request()
        request.name = turtle_name
        future = kill_client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_kill_service, turtle_name=turtle_name))

    # Kill Turtle Client Callback
    def callback_call_kill_service(self, future, turtle_name):
        try:
            future.result()
            for i, turtle in enumerate(self.alive_turtles):
                if turtle.name == turtle_name:
                    del self.alive_turtles[i]
                    
                    # publish alive_turtles
                    self.publish_alive_turtles()
                    break
        except Exception as e:
            self.get_logger().info("Service call failed %r" % (e,))

    
    # # ------------------    SPAWN TURTLE    ------------------
    
    # Function to Generate Random Coordinates
    def generate_random_coordinates(self):
        x_ = random.uniform(1.0, 9.0)
        y_ = random.uniform(1.0, 9.0)
        theta_ = random.uniform(0.0, 2*pi)
        return x_, y_ , theta_


    def call_spawn_turtle(self):
        turtle_name = self.turle_name_ + str(self.turtle_counter_)
        x , y, theta = self.generate_random_coordinates()
        self.spawn_turtle(x, y, theta, turtle_name)
        self.turtle_counter_ += 1
        pass
    
    # Spawn Turtle Client
    def spawn_turtle(self, x, y, theta, turtle_name ):
        client_ = self.create_client(Spawn, "spawn")
        while not client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for the Turtlesim Node Service...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_spawn_service, x=x, y=y, theta=theta))

    # Spawn Turtle Client Callback
    def callback_call_spawn_service(self, future, x, y, theta):
        try:
            response = future.result()
            turtle_name = response.name
            if turtle_name != "":
                self.get_logger().info("Turtle %s has been spawned successfully" % turtle_name)
                
                new_turtle = Turtle()
                new_turtle.x = x
                new_turtle.y = y
                new_turtle.theta = theta
                new_turtle.name = turtle_name
                self.alive_turtles.append(new_turtle)                
                self.publish_alive_turtles()
    
        except Exception as e:
            self.get_logger().info("Service call failed %r" % (e,))
            
    # --------------------------------------------------------  
 
 
def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
