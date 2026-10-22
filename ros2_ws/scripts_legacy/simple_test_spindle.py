#!/usr/bin/env python3
"""
Simple test script using moveit_commander to test spindle_tcp.
This is easier to use than the raw action interface.
"""

import rclpy
from rclpy.node import Node
import sys
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import RobotState

# Import moveit_commander
try:
    import moveit_commander
except ImportError:
    print("ERROR: moveit_commander not installed!")
    print("Install with: sudo apt install ros-humble-moveit-commander")
    sys.exit(1)


class SimpleSpindleTest(Node):
    def __init__(self):
        super().__init__('simple_spindle_test')
        
        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Create robot and scene objects
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Create move group for the arm
        # Change "arm" to "manipulator" if that's what your SRDF uses
        self.move_group_name = "arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.move_group_name)
        
        # Print robot information
        self.get_logger().info("=" * 50)
        self.get_logger().info("Robot Information:")
        self.get_logger().info(f"Planning frame: {self.move_group.get_planning_frame()}")
        self.get_logger().info(f"End effector link: {self.move_group.get_end_effector_link()}")
        self.get_logger().info(f"Available groups: {self.robot.get_group_names()}")
        self.get_logger().info("=" * 50)
        
        # Set planning parameters
        self.move_group.set_planning_time(5.0)
        self.move_group.set_num_planning_attempts(10)
        self.move_group.set_max_velocity_scaling_factor(0.1)
        self.move_group.set_max_acceleration_scaling_factor(0.1)
        
    def go_to_joint_state(self, joint_values):
        """Move to specific joint angles."""
        self.get_logger().info(f"Planning to joint state: {joint_values}")
        
        self.move_group.go(joint_values, wait=True)
        self.move_group.stop()
        
        self.get_logger().info("✅ Reached joint target")
        
    def go_to_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """Move spindle_tcp to a target pose."""
        pose_goal = Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        pose_goal.orientation.x = qx
        pose_goal.orientation.y = qy
        pose_goal.orientation.z = qz
        pose_goal.orientation.w = qw
        
        self.get_logger().info(f"Planning to pose: [{x:.3f}, {y:.3f}, {z:.3f}]")
        
        self.move_group.set_pose_target(pose_goal)
        
        # Plan the motion
        success, plan, planning_time, error_code = self.move_group.plan()
        
        if success:
            self.get_logger().info(f"✅ Planning successful! (took {planning_time:.2f}s)")
            
            # Execute the plan
            execute_input = input("Execute this plan? (y/n): ")
            if execute_input.lower() == 'y':
                self.move_group.execute(plan, wait=True)
                self.move_group.stop()
                self.move_group.clear_pose_targets()
                self.get_logger().info("✅ Execution complete!")
                return True
            else:
                self.get_logger().info("Execution skipped")
                self.move_group.clear_pose_targets()
                return False
        else:
            self.get_logger().error(f"❌ Planning failed! Error code: {error_code}")
            self.move_group.clear_pose_targets()
            return False
    
    def get_current_pose(self):
        """Get the current pose of the spindle_tcp."""
        current_pose = self.move_group.get_current_pose().pose
        self.get_logger().info("Current spindle_tcp pose:")
        self.get_logger().info(f"  Position: [{current_pose.position.x:.3f}, "
                             f"{current_pose.position.y:.3f}, "
                             f"{current_pose.position.z:.3f}]")
        self.get_logger().info(f"  Orientation: [{current_pose.orientation.x:.3f}, "
                             f"{current_pose.orientation.y:.3f}, "
                             f"{current_pose.orientation.z:.3f}, "
                             f"{current_pose.orientation.w:.3f}]")
        return current_pose
    
    def shutdown(self):
        moveit_commander.roscpp_shutdown()


def main():
    rclpy.init()
    
    node = SimpleSpindleTest()
    
    try:
        # Show current pose
        node.get_logger().info("\n" + "=" * 50)
        node.get_logger().info("CURRENT STATE")
        node.get_logger().info("=" * 50)
        node.get_current_pose()
        
        input("\nPress Enter to continue to Test 1...")
        
        # Test 1: Move to home position
        node.get_logger().info("\n" + "=" * 50)
        node.get_logger().info("TEST 1: Go to Home Position (all joints = 0)")
        node.get_logger().info("=" * 50)
        node.go_to_joint_state([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        input("\nPress Enter to continue to Test 2...")
        
        # Test 2: Move to a Cartesian position
        node.get_logger().info("\n" + "=" * 50)
        node.get_logger().info("TEST 2: Plan to Cartesian Position")
        node.get_logger().info("=" * 50)
        
        # Adjust these coordinates based on your robot's workspace
        node.go_to_pose(
            x=1.2,    # 1.2 meters in front
            y=0.0,    # centered
            z=1.0,    # 1 meter high
            qx=0.0,
            qy=0.707,  # 90° rotation around Y axis
            qz=0.0,
            qw=0.707
        )
        
        input("\nPress Enter to continue to Test 3...")
        
        # Test 3: Move to another position
        node.get_logger().info("\n" + "=" * 50)
        node.get_logger().info("TEST 3: Plan to Another Position")
        node.get_logger().info("=" * 50)
        
        node.go_to_pose(
            x=1.0,
            y=0.3,
            z=1.2,
            qw=1.0  # No rotation
        )
        
        # Test 4: Return to home
        input("\nPress Enter to return to home...")
        node.get_logger().info("\n" + "=" * 50)
        node.get_logger().info("TEST 4: Return to Home")
        node.get_logger().info("=" * 50)
        node.go_to_joint_state([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        node.get_logger().info("\n" + "=" * 50)
        node.get_logger().info("✅ All tests completed!")
        node.get_logger().info("=" * 50)
        
    except KeyboardInterrupt:
        node.get_logger().info("\nTest interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Error: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
