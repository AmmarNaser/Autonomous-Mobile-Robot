#! /usr/bin/env python3
from rclpy.node import Node
import rclpy # to use ros2 functionalities (we add rclpy as indepedencies in the package.xml file wile creating the  )
class mynode(Node):
    def __init__(self):
        super().__init__('pynodetest')
        self.counter_ = 0
        self.get_logger().info("get_logger functionused to print somthing")
        self.create_timer(0.5,self.timer_callback)
    
    
    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info("hello" + str(self.counter_))


def main(args=None):
    rclpy.init (args=args) #intialize ros2 comunication (you need to write each time you want to use ros2)
    node=mynode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()

    # in short what happened here inside our package we creat our file to contain the node which is a reusable seperated part of our app code 
    # we start by impoert rclpy and importing Node class 
    # then we define a function in which we pass no arguments and intialise the comunication and naming our node 
    # and write the function of this node which in our case print somethingand finally we shut down our comunication 