import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import numpy as np
import math

def artificial_field_potential(x,y,evader_x,evader_y,evader_theta,x_max,y_max):
        # Attraction caused by evader : field_constant * (field_weight) ^ distance
        fcon_evader = 1.2 # field_constant
        fwei_evader = -3 # field_weight
        att_evader = fwei_evader*(fcon_evader)**(-math.sqrt((x - (evader_x + 1*np.cos(evader_theta)))**2 + (y - (evader_y + 1*np.sin(evader_theta)))**2))

        # Repel caused by wall : field_constant * (field_weight) ^ -distance
        fcon_wall = 3 # field_constant
        fwei_wall = 1 # field_weight
        rep_wall_up = fwei_wall*(fcon_wall)**(y - y_max)
        rep_wall_down = fwei_wall*(fcon_wall)**(-y)
        rep_wall_right = fwei_wall*(fcon_wall)**(x - x_max)
        rep_wall_left = fwei_wall*(fcon_wall)**(-x)
        rep_wall = rep_wall_up + rep_wall_down + rep_wall_right + rep_wall_left

        
        # Repel caused by obstacle 1 : field_constant * (field_weight) ^ -distance
        obs1_pos_x = 4.0
        obs1_pos_y = 3.0
        obs1_rad = 1 + 0.2
        fcon_obs1 = 3 # field_constant
        fwei_obs1 = 1 # field_weight
        rep_obs1 = fwei_obs1*(fcon_obs1)**(obs1_rad-math.sqrt((x - obs1_pos_x)**2 + (y - obs1_pos_y)**2))

        # Repel caused by obstacle 2 : field_constant * (field_weight) ^ -distance
        obs2_pos_x = 7.5
        obs2_pos_y = 6.5
        obs2_rad = 1 + 0.2
        fcon_obs2 = 3 # field_constant
        fwei_obs2 = 1 # field_weight
        rep_obs2 = fwei_obs2*(fcon_obs2)**(obs2_rad-math.sqrt((x - obs2_pos_x)**2 + (y - obs2_pos_y)**2))

        # Repel caused by obstacle 3 : field_constant * (field_weight) ^ -distance
        obs3_pos_x = 2.5
        obs3_pos_y = 8.5
        obs3_rad = 1 + 0.2
        fcon_obs3 = 3 # field_constant
        fwei_obs3 = 1 # field_weight
        rep_obs3 = fwei_obs3*(fcon_obs3)**(obs3_rad-math.sqrt((x - obs3_pos_x)**2 + (y - obs3_pos_y)**2))

        return att_evader + rep_wall + rep_obs1 + rep_obs2 + rep_obs3

class TurtlePursuer(Node):
    def __init__(self):
        super().__init__("turtle_pursuer")

        # Publisher (send velocity to turtle2 & turtle1)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)

        # Subscriber (read position of turtle2)
        self.pose_subscriber_ = self.create_subscription(Pose,"/turtle2/pose", self.update_position, 10)

        # Subscriber (read position of turtle1)
        self.pose_subscriber_evader_ = self.create_subscription(Pose,"/turtle1/pose", self.update_position_evader, 10)

        # Timer to check the position and move the turtle every 0.1 seconds
        self.timer = self.create_timer(0.1, self.pursuit_evader)

        # Wall avoidance parameters
        self.wall_safe_distance = 1  # Distance to maintain from the wall
        self.x_min, self.x_max = 0.0, 11.0  # X boundaries of the turtlesim window
        self.y_min, self.y_max = 0.0, 11.0  # Y boundaries of the turtlesim window

    def update_position(self, pose:Pose):
        self.x = pose.x
        self.y = pose.y
        self.front_x = pose.x + (self.wall_safe_distance*np.cos(pose.theta))
        self.front_y = pose.y + (self.wall_safe_distance*np.sin(pose.theta))

        # left right "sensor" to detect APF
        self.FoV = (30/180)*np.pi # in rad
        # left
        self.front_left_x = pose.x + ((1/np.cos(self.FoV))*np.cos(pose.theta+self.FoV))
        self.front_left_y = pose.y + ((1/np.cos(self.FoV))*np.sin(pose.theta+self.FoV))
        # right
        self.front_right_x = pose.x + ((1/np.cos(self.FoV))*np.cos(pose.theta-self.FoV))
        self.front_right_y = pose.y + ((1/np.cos(self.FoV))*np.sin(pose.theta-self.FoV))

    def update_position_evader(self, pose:Pose):
        self.poseE_x = pose.x
        self.poseE_y = pose.y
        self.poseE_theta = pose.theta

    def pursuit_evader(self): 
        cmd = Twist()
        dist = math.sqrt((self.x - self.poseE_x)**2 + (self.y - self.poseE_y)**2)
        if dist < 0.7:
            cmd.linear.x = 0.0
            self.get_logger().info("Captured!")
            while True:
                cmd.linear.x = 0.0
        else:
            cmd.linear.x = 1.5
        
        self.APF_front = artificial_field_potential(self.front_x,self.front_y,self.poseE_x,self.poseE_y,self.poseE_theta,self.x_max,self.y_max)
        self.APF_left = artificial_field_potential(self.front_left_x,self.front_left_y,self.poseE_x,self.poseE_y,self.poseE_theta,self.x_max,self.y_max)
        self.APF_right = artificial_field_potential(self.front_right_x,self.front_right_y,self.poseE_x,self.poseE_y,self.poseE_theta,self.x_max,self.y_max)
        self.APF_current = artificial_field_potential(self.x,self.y,self.poseE_x,self.poseE_y,self.poseE_theta,self.x_max,self.y_max)

        if self.APF_left < self.APF_front and self.APF_left < self.APF_right and self.APF_left < self.APF_current :
            cmd.angular.z = 1.0
        elif self.APF_right < self.APF_front and self.APF_right < self.APF_left and self.APF_right < self.APF_current :
            cmd.angular.z = -1.0
        elif self.APF_current < self.APF_front and self.APF_left < self.APF_right :
            cmd.angular.z = 3.0
        elif self.APF_current < self.APF_front and self.APF_right < self.APF_left :
            cmd.angular.z = -3.0
        else:
            cmd.angular.z = 0.0

        self.cmd_vel_publisher_.publish(cmd)

def main():
    rclpy.init()
    node = TurtlePursuer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()