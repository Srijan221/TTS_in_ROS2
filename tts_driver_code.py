import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import yaml
import os
import threading


class Nav2GoalPublisher(Node):
    def __init__(self):
        super().__init__('nav2_goal_publisher')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Load the goal positions from the YAML file
        yaml_file = os.path.join(os.path.dirname(__file__), 'goals.yaml')
        with open(yaml_file, 'r') as file:
            self.goals = yaml.safe_load(file)['positions']
        
        self.current_goal_handle = None

    def publish_goal(self, position_name):
        if position_name not in self.goals:
            self.get_logger().error(f"Position '{position_name}' not found in the YAML file.")
            return

        goal = self.goals[position_name]
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = "map"  # Common frame of reference
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = goal['x']
        goal_msg.pose.pose.position.y = goal['y']
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion (yaw only, assuming roll and pitch are 0)
        qz = sin(goal['theta'] / 2.0)
        qw = cos(goal['theta'] / 2.0)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(f"Going to '{position_name}'")

        if self.current_goal_handle:
            self.cancel_goal()

        self.send_goal(goal_msg)

    def send_goal(self, goal_msg):
        self.action_client.wait_for_server()
        self.current_goal_handle = self.action_client.send_goal_async(goal_msg)
        self.current_goal_handle.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected')
            self.current_goal_handle = None
            return
        self.get_logger().info('Goal accepted')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        self.get_logger().info('Goal finished with status: {0}'.format(result.status))
        self.current_goal_handle = None

    def cancel_goal(self):
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
            self.get_logger().info('Current goal canceled')

def input_thread(node):
    while rclpy.ok():
        try:
            user_input = input("Enter the position name (e.g., 'position_1'): ").strip()
            if user_input:
                node.publish_goal(user_input)
        except EOFError:
            break

def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalPublisher()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    input_thread_ = threading.Thread(target=input_thread, args=(node,))
    input_thread_.start()

    try:
        executor.spin()
    finally:
        executor.shutdown()
        input_thread_.join()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
