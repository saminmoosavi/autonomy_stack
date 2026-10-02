#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    MoveItErrorCodes,
)
from shape_msgs.msg import SolidPrimitive
import time

class XArmCartesianPlanner(Node):

    def __init__(self):
        super().__init__('xarm_cartesian_planner')

        self.move_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        self.execute_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory'
        )

    def plan(self, x, y, z):

        self.get_logger().info('Waiting for MoveGroup...')

        if not self.move_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('/move_action not available')
            return None

        goal = MoveGroup.Goal()

        goal.request.group_name = 'xarm6'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0

        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2

        # IMPORTANT:
        # We explicitly plan only here.
        goal.planning_options.plan_only = True

        ref_frame = 'link_base'
        eef_link = 'link_eef'

        # -------------------------
        # Position constraint
        # -------------------------

        position_constraint = PositionConstraint()

        position_constraint.header.frame_id = ref_frame
        position_constraint.link_name = eef_link
        position_constraint.weight = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]

        target_pose = PoseStamped()
        target_pose.header.frame_id = ref_frame

        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z

        position_constraint.constraint_region.primitives.append(sphere)
        position_constraint.constraint_region.primitive_poses.append(
            target_pose.pose
        )

        # -------------------------
        # Orientation constraint
        # -------------------------

        orientation_constraint = OrientationConstraint()

        orientation_constraint.header.frame_id = ref_frame
        orientation_constraint.link_name = eef_link

        orientation_constraint.orientation.x = 0.0
        orientation_constraint.orientation.y = -1.0
        orientation_constraint.orientation.z = 0.0
        orientation_constraint.orientation.w = 0.0

        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1

        orientation_constraint.weight = 1.0

        # -------------------------
        # Goal constraints
        # -------------------------

        constraints = Constraints()
        constraints.name = 'pose_goal'

        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        goal.request.goal_constraints.append(constraints)



        self.get_logger().info(
            f'Planning to x={x:.3f}, y={y:.3f}, z={z:.3f}'
        )

        send_future = self.move_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Planning goal rejected')
            return None

        self.get_logger().info('Planning goal accepted')

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()

        if result is None:
            self.get_logger().error('No planning result received')
            return None

        move_result = result.result

        self.get_logger().info(
            f'MoveIt planning error code: {move_result.error_code.val}'
        )

        if move_result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error('Planning failed')
            return None

        trajectory = move_result.planned_trajectory

        num_points = len(trajectory.joint_trajectory.points)

        self.get_logger().info(
            f'Planning succeeded: {num_points} trajectory points'
        )

        return trajectory

    def execute(self, trajectory):

        self.get_logger().info('Waiting for /execute_trajectory...')

        if not self.execute_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                '/execute_trajectory action server unavailable'
            )
            return False

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info('Sending trajectory for execution...')

        send_future = self.execute_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Execution goal rejected')
            return False

        self.get_logger().info(
            'Execution accepted. Waiting for Gazebo motion...'
        )

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()

        if result is None:
            self.get_logger().error('No execution result')
            return False

        error_code = result.result.error_code.val

        self.get_logger().info(
            f'Execution MoveIt error code: {error_code}'
        )

        if error_code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('Execution succeeded')
            return True

        self.get_logger().error('Execution failed')
        return False


def main():


    rclpy.init()

    planner = XArmCartesianPlanner()

    goal_points = [
        (0.566,  -0.174, 0.315),
        (0.652,  0.0043, 0.222),
        (0.432,  0.200, 0.247),
        # (0.377,  -0.220, -0.066),
    ]

    for i, (x, y, z) in enumerate(goal_points):

        planner.get_logger().info(
            f'===== Goal {i + 1}/{len(goal_points)} ====='
        )

        planner.get_logger().info(
            f'Target: x={x:.3f}, y={y:.3f}, z={z:.3f}'
        )

        trajectory = planner.plan(
            x=x,
            y=y,
            z=z
        )

        if trajectory is None:
            planner.get_logger().error(
                f'Planning failed at goal {i + 1}. Stopping.'
            )
            break

        success = planner.execute(trajectory)

        if not success:
            planner.get_logger().error(
                f'Execution failed at goal {i + 1}. Stopping.'
            )
            break

        planner.get_logger().info(
            f'Goal {i + 1} reached successfully.'
        )
        time.sleep(1.0)

    else:
        planner.get_logger().info(
            'All goal points reached successfully.'
        )

    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()