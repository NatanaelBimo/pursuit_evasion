import math
import random
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TurtleEvader(Node):
    def __init__(self):
        super().__init__('evader_turtle')

        # Declare the pursuer turtle name
        self.pursuer_turtle = self.declare_parameter(
            'pursuer_turtle', 'turtlePursuer'
        ).get_parameter_value().string_value

        # Buffer and listener for TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher to send velocity commands to the evader (turtle1)
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # Subscribe to turtle1's pose to monitor its position (for wall avoidance)
        self.create_subscription(Pose, 'turtle1/pose', self.update_position, 10)

        # Timer to check the position and move the turtle every 0.1 seconds
        self.timer = self.create_timer(0.1, self.evade_pursuer)

        # Wall avoidance parameters
        self.wall_safe_distance = .5  # Distance to maintain from the wall
        self.x_min, self.x_max = 0.0, 11.0  # X boundaries of the turtlesim window
        self.y_min, self.y_max = 0.0, 11.0  # Y boundaries of the turtlesim window

        # Current position of the evader
        self.evader_x = 5.5
        self.evader_y = 5.5

        # Reverse mechanism state
        self.reversing = False
        self.reverse_counter = 0  # How long to reverse
        self.max_reverse_steps = 10  # Maximum time steps to reverse

        # Random movement parameters
        self.sharp_turn_chance = 0.1  # 10% chance of sharp turn
        self.random_motion_counter = 0
        self.exploration_threshold = 5.0  # Evader explores if far enough from pursuer

    def update_position(self, msg: Pose):
        """Update the evader's current position for wall detection."""
        self.evader_x = msg.x
        self.evader_y = msg.y

    def evade_pursuer(self):
        # If the turtle is currently reversing, continue that behavior
        if self.reversing:
            self.reverse_turtle()
            return

        # Frame names for evader and pursuer
        from_frame_rel = self.pursuer_turtle
        to_frame_rel = 'turtle1'  # The evader is now 'turtle1'

        try:
            # Lookup the transform between pursuer and evader
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        # Calculate the distance and angle to the pursuer
        distance_to_pursuer = math.sqrt(
            t.transform.translation.x ** 2 +
            t.transform.translation.y ** 2)

        # Create a Twist message to control turtle1's velocity
        msg = Twist()

        # If the evader is far from the pursuer, explore the environment
        if distance_to_pursuer > self.exploration_threshold:
            self.explore(msg)
        else:
            self.evade(msg, t)

        # Check if the turtle is near any wall and adjust direction
        if self.avoid_walls(msg):
            # If the wall is near, initiate reverse behavior
            self.reversing = True
            self.reverse_counter = self.max_reverse_steps
            return

        # Publish the velocity command if it's safe to move
        self.publisher.publish(msg)

    def evade(self, msg, t):
        """Evade the pursuer with randomized movements."""
        # Calculate the angle away from the pursuer
        angle_away_from_pursuer = math.atan2(
            -t.transform.translation.y,
            -t.transform.translation.x)

        # Randomly apply sharp turns for more dynamic motion
        if random.random() < self.sharp_turn_chance:
            msg.angular.z = random.uniform(-3.0, 3.0)  # Sharp turn
            msg.linear.x = random.uniform(0.5, 1.5)  # Adjust speed
        else:
            # Move in the opposite direction from the pursuer with slight variation
            msg.angular.z = angle_away_from_pursuer + random.uniform(-0.5, 0.5)
            msg.linear.x = 1.0  # Linear speed to move away

    def explore(self, msg):
        """Explore the environment randomly when far from the pursuer."""
        if self.random_motion_counter <= 0:
            # Set random angular and linear velocities
            msg.angular.z = random.uniform(-1.0, 1.0)
            msg.linear.x = random.uniform(0.5, 2.0)
            # Set how long to continue this motion
            self.random_motion_counter = random.randint(10, 50)
        else:
            # Continue current random movement
            self.random_motion_counter -= 1

    def avoid_walls(self, msg):
        """Detect walls and adjust movement to bounce away from them."""
        # Check if the turtle is near the left or right boundary
        if self.evader_x < self.x_min + self.wall_safe_distance or self.evader_x > self.x_max - self.wall_safe_distance:
            return True  # Wall detected

        # Check if the turtle is near the top or bottom boundary
        if self.evader_y < self.y_min + self.wall_safe_distance or self.evader_y > self.y_max - self.wall_safe_distance:
            return True  # Wall detected

        return False

    def reverse_turtle(self):
        """Make the turtle reverse when it hits a wall."""
        msg = Twist()

        # If we are still reversing, continue moving backward
        if self.reverse_counter > 0:
            msg.linear.x = -.5  # Move backward
            msg.angular.z = random.uniform(-2.5, 2.5)  # Turn randomly while reversing
            self.reverse_counter -= 1
        else:
            # Stop reversing after a set amount of time
            self.reversing = False
            msg.linear.x = 0.0
            msg.angular.z = random.uniform(-1.0, 1.0)  # Random turn to find a new path

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = TurtleEvader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
