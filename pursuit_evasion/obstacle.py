import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Pose as GeometryPose

class ObstacleSpawner(Node):

    def __init__(self):
        super().__init__('obstacle_spawner')
        self.cli = self.create_client(Spawn, 'spawn')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting...')

        # Obstacle positions
        self.obstacle_positions = [
            ('turtle_obstacle_1', 2.0, 9.0, 0.0),
            ('turtle_obstacle_2', 9.0, 9.0, 0.0),
            ('turtle_obstacle_3', 5.5, 2.0, 0.0),
        ]

        # Publishers for the positions of the obstacles
        self.obstacle_publishers = []
        for obstacle_name, x, y, theta in self.obstacle_positions:
            pub = self.create_publisher(GeometryPose, f'/{obstacle_name}/pose', 10)
            self.obstacle_publishers.append(pub)

        # Spawn turtles and publish positions
        self.spawn_turtles()

        # Create a timer to periodically publish obstacle positions
        self.timer = self.create_timer(0.1, self.publish_obstacle_positions)

    def spawn_turtles(self):
        for name, x, y, theta in self.obstacle_positions:
            self.call_spawn_service(name, x, y, theta)

    def call_spawn_service(self, name, x, y, theta):
        req = Spawn.Request()
        req.name = name
        req.x = x
        req.y = y
        req.theta = theta

        future = self.cli.call_async(req)
        future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Successfully spawned {response.name}')
        except Exception as e:
            self.get_logger().error(f'Service call failed {e}')

    def publish_obstacle_positions(self):
        """Publish the positions of the obstacles."""
        for idx, (name, x, y, theta) in enumerate(self.obstacle_positions):
            msg = GeometryPose()
            msg.position.x = x
            msg.position.y = y
            msg.orientation.z = theta
            self.obstacle_publishers[idx].publish(msg)

def main():
    rclpy.init()
    node = ObstacleSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
