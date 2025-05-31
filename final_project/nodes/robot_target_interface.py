#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from mage_msgs.action import RobotTarget
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from rclpy.action.server import ServerGoalHandle
import math


class RobotTargetServer(Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name)

        self._action_server = ActionServer(
            node=self,  # Node
            action_type=RobotTarget,  # Action type
            action_name="reach_target",  # Action server name
            execute_callback=self.execute_cb,  # Execute callback
            goal_callback=self.goal_cb,  # Goal callback
        )

        # Publisher to publish velocity commands
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscriber to get odometry data
        # We are using a MutuallyExclusiveCallbackGroup to ensure
        # that the callback is executed in a separate thread from the main thread
        # qos_profile = QoSProfile(depth=10)
        self._odometry_cbg = MutuallyExclusiveCallbackGroup()
        self._odometry_sub = self.create_subscription(
            Odometry,
            "odom",
            self.odometry_cb,
            10,
            callback_group=self._odometry_cbg,
        )

        # Initialize variables
        self._goal_x = 0.0
        self._goal_y = 0.0
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._goal_reached = False

        # Proportional control gains
        self._kl = 0.5  # Linear velocity gain
        self._ka = 0.8  # Angular velocity gain

        # Print the server is available
        output = "\n============================================="
        output += "\nAction Server 'reach_target' is available"
        output += "\n============================================="

        self.get_logger().info(output)

    def stop_robot(self):
        """Central method to stop the robot."""
        self.get_logger().info("Stopping the robot.")
        stop_twist = Twist()
        self._cmd_vel_pub.publish(stop_twist)

    def goal_cb(self, goal_request: RobotTarget.Goal):
        """
        Callback to handle new goal requests

        Args:
            goal_request (RobotTarget.Goal): Goal request

        Returns:
            GoalResponse.ACCEPT: Accept the goal
        """
        self.get_logger().info("Received new goal request")
        self._goal_x = goal_request.target.x
        self._goal_y = goal_request.target.y
        self._goal_reached = False

        # Let the action client know that the goal is accepted
        return GoalResponse.ACCEPT

    def odometry_cb(self, msg: Odometry):
        """
        Callback to handle odometry data

        Args:
            msg (Odometry): Odometry message
        """

        # position
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

        # orientation
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

        # Convert quaternion to Euler angles
        _, _, self._robot_yaw = euler_from_quaternion(quaternion)

    def execute_cb(self, goal_handle: ServerGoalHandle):
        """
        Callback to execute the goal

        Args:
            goal_handle (ServerGoalHandle): Goal handle

        Returns:
            RobotTarget.Result: Result of the goal execution
        """
        if self._goal_reached:
            self.get_logger().info("Goal already reached.")
            return RobotTarget.Result(success=True, message="Goal already reached")

        self.get_logger().info(f"Executing goal: ({self._goal_x},{self._goal_y})")
        # Create a rate object to control the loop rate
        rate = self.create_rate(10)

        while rclpy.ok():  # While ROS is running
            distance_to_goal = self.compute_distance_to_goal()

            if distance_to_goal < 0.03:
                self.get_logger().info(">>>>>>>>Goal reached!")
                self.stop_robot()
                self._goal_reached = True
                return RobotTarget.Result(success=True, message="Goal reached")

            self.adjust_robot_motion(distance_to_goal, goal_handle)
            rate.sleep()

    def compute_distance_to_goal(self):
        """
        Compute the distance to the goal

        Returns:
            float: Distance to the goal
        """
        return math.sqrt(
            (self._goal_x - self._robot_x) ** 2 + (self._goal_y - self._robot_y) ** 2
        )

    def adjust_robot_motion(self, distance_to_goal, goal_handle: ServerGoalHandle):
        """
        Adjust the robot motion based on the distance to the goal

        Args:
            distance_to_goal (float): Distance to the goal
            goal_handle (ServerGoalHandle): Goal handle
        """
        dy = self._goal_y - self._robot_y
        dx = self._goal_x - self._robot_x
        # Compute angle to goal
        angle_to_goal = math.atan2(dy, dx)
        # Compute angular velocity
        angular_velocity = self.calculate_angular_velocity(angle_to_goal)
        # Proportional control for linear velocity
        linear_velocity = min(self._kl * distance_to_goal, 0.6)

        # Publish velocity command
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self._cmd_vel_pub.publish(twist)

        # Publish distance to goal as feedback
        feedback_msg = RobotTarget.Feedback()
        feedback_msg.distance_to_goal = distance_to_goal
        goal_handle.publish_feedback(feedback_msg)

    def calculate_angular_velocity(self, angle_to_goal):
        """
        Calculate the angular velocity based on the angle to the goal

        Args:
            angle_to_goal (float): Angle to the goal

        Returns:
            float: Angular velocity
        """
        # Adjust angle to ensure proper rotation direction
        if angle_to_goal < 0:
            angle_to_goal += 2 * math.pi

        # Compute relative orientation between robot and goal
        w = angle_to_goal - self._robot_yaw
        if w > math.pi:  # Ensure shortest rotation
            w -= 2 * math.pi
        elif w < -math.pi:  # Ensure shortest rotation
            w += 2 * math.pi

        # Proportional control for angular velocity, with limit for turtlebot max angular velocity
        angular_velocity = self._ka * w
        return max(min(angular_velocity, 1.82), -1.82)


def main(args=None):
    rclpy.init(args=args)
    node = RobotTargetServer("robot_target_server")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT) detected")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()