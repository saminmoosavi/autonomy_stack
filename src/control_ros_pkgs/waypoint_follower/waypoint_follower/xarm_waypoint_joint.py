#!/usr/bin/env python3

import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes


class XArmJointPlanner(Node):

    def __init__(self):
        super().__init__('xarm_joint_planner')

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

    def plan(self, joint_positions):

        if len(joint_positions) != 6:
            self.get_logger().error(
                f'Expected 6 joint values, got {len(joint_positions)}'
            )
            return None

        self.get_logger().info(
            'Waiting for /move_action...'
        )

        if not self.move_client.wait_for_server(
            timeout_sec=10.0
        ):
            self.get_logger().error(
                '/move_action not available'
            )
            return None

        goal = MoveGroup.Goal()

        # MoveIt planning group
        goal.request.group_name = 'xarm6'

        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0

        # Slow motion
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.10

        # Only plan here.
        # Execution is handled separately below.
        goal.planning_options.plan_only = True

        # --------------------------------------------------
        # Joint constraints
        # --------------------------------------------------

        joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6'
        ]

        constraints = Constraints()
        constraints.name = 'joint_goal'

        for joint_name, position in zip(
            joint_names,
            joint_positions
        ):

            joint_constraint = JointConstraint()

            joint_constraint.joint_name = joint_name

            # Joint position is in radians
            joint_constraint.position = float(position)

            # Allowed error around target position
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01

            joint_constraint.weight = 1.0

            constraints.joint_constraints.append(
                joint_constraint
            )

        goal.request.goal_constraints.append(
            constraints
        )

        self.get_logger().info(
            'Planning to joint goal:'
        )

        for name, position in zip(
            joint_names,
            joint_positions
        ):
            self.get_logger().info(
                f'  {name}: {position:.4f} rad'
            )

        # --------------------------------------------------
        # Send planning request
        # --------------------------------------------------

        send_future = self.move_client.send_goal_async(
            goal
        )

        rclpy.spin_until_future_complete(
            self,
            send_future
        )

        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(
                'Failed to send planning goal'
            )
            return None

        if not goal_handle.accepted:
            self.get_logger().error(
                'Planning goal rejected'
            )
            return None

        self.get_logger().info(
            'Planning goal accepted'
        )

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result = result_future.result()

        if result is None:
            self.get_logger().error(
                'No planning result received'
            )
            return None

        move_result = result.result

        error_code = move_result.error_code.val

        self.get_logger().info(
            f'MoveIt planning error code: {error_code}'
        )

        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                'Planning failed'
            )
            return None

        trajectory = move_result.planned_trajectory

        num_points = len(
            trajectory.joint_trajectory.points
        )

        self.get_logger().info(
            f'Planning succeeded: '
            f'{num_points} trajectory points'
        )

        if num_points > 0:
            last_point = trajectory.joint_trajectory.points[-1]

            duration = (
                last_point.time_from_start.sec
                + last_point.time_from_start.nanosec * 1e-9
            )

            self.get_logger().info(
                f'Planned trajectory duration: '
                f'{duration:.2f} seconds'
            )

        return trajectory

    def execute(self, trajectory):

        self.get_logger().info(
            'Waiting for /execute_trajectory...'
        )

        if not self.execute_client.wait_for_server(
            timeout_sec=10.0
        ):
            self.get_logger().error(
                '/execute_trajectory not available'
            )
            return False

        goal = ExecuteTrajectory.Goal()

        goal.trajectory = trajectory

        self.get_logger().info(
            'Sending trajectory for execution...'
        )

        send_future = self.execute_client.send_goal_async(
            goal
        )

        rclpy.spin_until_future_complete(
            self,
            send_future
        )

        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(
                'Failed to send execution goal'
            )
            return False

        if not goal_handle.accepted:
            self.get_logger().error(
                'Execution goal rejected'
            )
            return False

        self.get_logger().info(
            'Execution accepted'
        )

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result = result_future.result()

        if result is None:
            self.get_logger().error(
                'No execution result received'
            )
            return False

        error_code = result.result.error_code.val

        if error_code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(
                'Execution succeeded'
            )
            return True

        self.get_logger().error(
            f'Execution failed. '
            f'MoveIt error code: {error_code}'
        )

        return False


def main():

    rclpy.init()

    planner = XArmJointPlanner()

    # --------------------------------------------------
    # Joint goals
    #
    # Order:
    #
    # joint1
    # joint2
    # joint3
    # joint4
    # joint5
    # joint6
    #
    # ALL VALUES ARE IN RADIANS
    # --------------------------------------------------

    joint_goals = [

        [-0.02618, 0.46775, -1.15366, -0.02443, 0.65101, -0.03840],

        [
            0.3,
            -0.6,
            -0.9,
            0.2,
            1.1,
            0.3
        ],

        [
            0.5,
            -0.4,
            -0.7,
            0.0,
            1.0,
            0.5
        ],

    ]

    # --------------------------------------------------
    # Sequentially execute goals
    # --------------------------------------------------

    for i, joint_goal in enumerate(
        joint_goals
    ):

        planner.get_logger().info(
            '========================================'
        )

        planner.get_logger().info(
            f'Joint Goal '
            f'{i + 1}/{len(joint_goals)}'
        )

        planner.get_logger().info(
            '========================================'
        )

        # Plan
        trajectory = planner.plan(
            joint_goal
        )

        if trajectory is None:

            planner.get_logger().error(
                f'Planning failed at '
                f'goal {i + 1}. Stopping.'
            )

            break

        # Execute
        success = planner.execute(
            trajectory
        )

        if not success:

            planner.get_logger().error(
                f'Execution failed at '
                f'goal {i + 1}. Stopping.'
            )

            break

        planner.get_logger().info(
            f'Joint goal {i + 1} '
            f'reached successfully.'
        )

        # --------------------------------------------------
        # Pause at each goal
        # --------------------------------------------------

        planner.get_logger().info(
            'Pausing for 1 second...'
        )

        time.sleep(1.0)

    else:

        planner.get_logger().info(
            'All joint goals reached successfully.'
        )

    planner.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
