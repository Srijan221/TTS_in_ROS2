import rclpy
from rclpy.node import Node
from stretch_nav2.robot_navigator import BasicNavigator, Pose
import yaml
import os
import threading


class Nav2GoalPublisher(Node):
    def __init__(self):
        super().__init__('nav2_goal_publisher')
        self.navigator = BasicNavigator()
        
        # Load the goal positions from the YAML file
        yaml_file = os.path.join(os.path.dirname(__file__), 'goals.yaml')
        with open(yaml_file, 'r') as file:
            self.goals = yaml.safe_load(file)['positions']
        
        self.current_goal_name = None

    def publish_goal(self, position_name):
        if position_name not in self.goals:
            self.get_logger().error(f"Position '{position_name}' not found in the YAML file.")
            return

        goal = self.goals[position_name]
        goal_pose = Pose()
        goal_pose.position.x = goal['x']
        goal_pose.position.y = goal['y']

        # Convert theta to quaternion (yaw only, assuming roll and pitch are 0)
        from math import sin, cos
        qz = sin(goal['theta'] / 2.0)
        qw = cos(goal['theta'] / 2.0)
        goal_pose.orientation.z = qz
        goal_pose.orientation.w = qw

        self.get_logger().info(f"Going to '{position_name}'")

        if self.current_goal_name:
            self.cancel_goal()

        self.navigator.go_to_pose(goal_pose)
        self.current_goal_name = position_name

    def cancel_goal(self):
        self.navigator.cancel_goal()
        self.get_logger().info(f"Canceled current goal '{self.current_goal_name}'")
        self.current_goal_name = None

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

    input_thread_ = threading.Thread(target=input_thread, args=(node,))
    input_thread_.start()

    rclpy.spin(node)

    input_thread_.join()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
