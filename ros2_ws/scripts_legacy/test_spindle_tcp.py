#!/usr/bin/env python3
"""
Test script to verify MoveIt can plan to the spindle_tcp.
This script demonstrates how to use MoveIt2 to plan motions using the spindle as the TCP.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive
import sys


class TestSpindleTCP(Node):
    def __init__(self):
        super().__init__('test_spindle_tcp')
        
        # Create action client for MoveGroup
        self._action_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'  # Default MoveGroup action namespace
        )
        
        self.get_logger().info("Waiting for MoveGroup action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Connected to MoveGroup action server!")
        
    def create_pose_goal(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """Create a pose goal for the spindle_tcp."""
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        # Orientation (quaternion)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        
        return pose
    
    def plan_to_pose(self, target_pose, group_name="arm"):
        """Plan a motion to the target pose using the spindle_tcp as end effector."""
        
        # Create the motion plan request
        goal_msg = MoveGroup.Goal()
        
        # Set up the planning request
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        
        # Define workspace boundaries (adjust based on your robot)
        goal_msg.request.workspace_parameters.min_corner.x = -3.0
        goal_msg.request.workspace_parameters.min_corner.y = -3.0
        goal_msg.request.workspace_parameters.min_corner.z = -3.0
        goal_msg.request.workspace_parameters.max_corner.x = 3.0
        goal_msg.request.workspace_parameters.max_corner.y = 3.0
        goal_msg.request.workspace_parameters.max_corner.z = 3.0
        
        # Set the planning group
        goal_msg.request.group_name = group_name
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        
        # Create position constraint for spindle_tcp
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "base_link"
        position_constraint.link_name = "spindle_tcp"  # IMPORTANT: Use spindle_tcp
        
        # Define the target region (small box around target position)
        position_constraint.constraint_region.primitives.append(SolidPrimitive())
        position_constraint.constraint_region.primitives[0].type = SolidPrimitive.BOX
        position_constraint.constraint_region.primitives[0].dimensions = [0.001, 0.001, 0.001]
        
        # Set the target position
        position_constraint.constraint_region.primitive_poses.append(Pose())
        position_constraint.constraint_region.primitive_poses[0].position = target_pose.pose.position
        position_constraint.constraint_region.primitive_poses[0].orientation.w = 1.0
        
        position_constraint.weight = 1.0
        
        # Create orientation constraint for spindle_tcp
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "base_link"
        orientation_constraint.link_name = "spindle_tcp"  # IMPORTANT: Use spindle_tcp
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.01
        orientation_constraint.absolute_y_axis_tolerance = 0.01
        orientation_constraint.absolute_z_axis_tolerance = 0.01
        orientation_constraint.weight = 1.0
        
        # Add constraints to goal
        goal_msg.request.goal_constraints.append(Constraints())
        goal_msg.request.goal_constraints[0].position_constraints.append(position_constraint)
        goal_msg.request.goal_constraints[0].orientation_constraints.append(orientation_constraint)
        
        # Set planning options
        goal_msg.planning_options.plan_only = False  # Set to True if you only want to plan, not execute
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        
        # Send the goal
        self.get_logger().info(f"Planning to position: [{target_pose.pose.position.x:.3f}, "
                             f"{target_pose.pose.position.y:.3f}, "
                             f"{target_pose.pose.position.z:.3f}]")
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected!")
            return False
        
        self.get_logger().info("Goal accepted, waiting for result...")
        
        # Wait for the result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        
        if result.error_code.val == 1:  # MoveItErrorCodes.SUCCESS
            self.get_logger().info("✅ Planning and execution successful!")
            return True
        else:
            self.get_logger().error(f"❌ Planning failed with error code: {result.error_code.val}")
            return False


def main(args=None):
    rclpy.init(args=args)
    
    node = TestSpindleTCP()
    
    try:
        # Test 1: Plan to a reachable position
        node.get_logger().info("\n=== Test 1: Planning to position [1.0, 0.0, 1.2] ===")
        target_pose = node.create_pose_goal(
            x=1.0, 
            y=0.0, 
            z=1.2,
            qx=0.0,
            qy=0.707,  # 90 degrees rotation around Y
            qz=0.0,
            qw=0.707
        )
        success = node.plan_to_pose(target_pose)
        
        if success:
            node.get_logger().info("Test 1 PASSED ✅")
        else:
            node.get_logger().error("Test 1 FAILED ❌")
        
        # Test 2: Plan to another position
        input("\nPress Enter to run Test 2...")
        node.get_logger().info("\n=== Test 2: Planning to position [0.8, 0.3, 1.5] ===")
        target_pose = node.create_pose_goal(
            x=0.8,
            y=0.3,
            z=1.5,
            qw=1.0  # No rotation
        )
        success = node.plan_to_pose(target_pose)
        
        if success:
            node.get_logger().info("Test 2 PASSED ✅")
        else:
            node.get_logger().error("Test 2 FAILED ❌")
            
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Test failed with exception: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
