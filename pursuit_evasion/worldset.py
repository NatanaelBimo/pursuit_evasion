import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from functools import partial

class TurtleSpawn(Node):
    def __init__(self):
        super().__init__("turtle_spawn")
        self.spawn_turtle(3.0, 8.0, 0.0,'obs1')
        self.spawn_turtle(9.0, 8.0, 0.0,'obs2')
        self.spawn_turtle(5.5, 4.0, 0.0,'obs3')
        self.spawn_turtle(0.0,0.0,0.0,'turtle2')
    
    def spawn_turtle(self, x, y, theta, name):
        client = self.create_client(Spawn, '/spawn')
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for service...")
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_spawn))

    def callback_spawn(self,future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))


class TurtleObstacleMove(Node):
    def __init__(self):
        super().__init__("turtle_obstacle_move")
        # Publisher (send velocity to obstacles)
        self.cmd_vel_publisher1_ = self.create_publisher(Twist, "/obs1/cmd_vel", 20)
        self.cmd_vel_publisher2_ = self.create_publisher(Twist, "/obs2/cmd_vel", 20)
        self.cmd_vel_publisher3_ = self.create_publisher(Twist, "/obs3/cmd_vel", 20)
        self.timer_ = self.create_timer(0.1, self.send_velocity_command)

    def send_velocity_command(self):
        cmd1 = Twist()
        cmd2 = Twist()
        cmd3 = Twist()
        cmd1.linear.x = 2.0
        cmd1.angular.z = -2.0
        cmd2.linear.x = 2.0
        cmd2.angular.z = -2.0
        cmd3.linear.x = 2.0
        cmd3.angular.z = -2.0
        self.cmd_vel_publisher1_.publish(cmd1)
        self.cmd_vel_publisher2_.publish(cmd2)
        self.cmd_vel_publisher3_.publish(cmd3)

def main():
    rclpy.init()
    node = TurtleSpawn()
    node1 = TurtleObstacleMove()
    try:
        rclpy.spin(node1)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()