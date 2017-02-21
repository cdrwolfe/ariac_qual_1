#!/usr/bin/env python
import sys, tf
import time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
import dynamic_reconfigure.client


# Arm controller module, handles functions associated with arm control, tasks etc
class ArmController:
    # An arm controller class
    def __init__(self, operators):
        # Initialise
        self.op = operators
        self.pastArmState = None
        # We need a tf listener to convert poses into arm reference base
        self.tf_listener = tf.TransformListener()
        # Initialise moveit commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        # Create move group for manipulator
        self.groupManipulator = moveit_commander.MoveGroupCommander("manipulator")
        # Allow replanning to increase the odds of a solution
        self.groupManipulator.allow_replanning(True)
        # Allow 5 seconds per planning attempt
        self.groupManipulator.set_planning_time(5)
        # Set goal joint tolerance
        self.groupManipulator.set_goal_joint_tolerance(0.005)
        # Set goal goal tolerance
        self.groupManipulator.set_goal_tolerance(0.005)
        # Set goal goal tolerance
        self.groupManipulator.set_goal_position_tolerance(0.005)
        # Set goal orientation tolerance
        self.groupManipulator.set_goal_orientation_tolerance(0.005)
        # Define end effector group
        self.groupEffector = moveit_commander.MoveGroupCommander("endeffector")
        # Set trajectory execution ros parameters to disabled
        client = dynamic_reconfigure.client.Client('move_group/trajectory_execution/')
        params = { 'allowed_start_tolerance' : '0.0'}
        config = client.update_configuration(params)

    def armPose(self):
        # Return current arm pose
        pose = self.groupManipulator.get_current_pose()
        # Return
        return pose

    def poseTarget(self, target):
        # Given a target try and position arm end effector over it, return bool
        poseTarget = geometry_msgs.msg.Pose()
        poseTarget.position.x = target.pose.position.x
        poseTarget.position.y = target.pose.position.y
        poseTarget.position.z = target.pose.position.z
        poseTarget.orientation = target.pose.orientation
        self.groupManipulator.set_pose_target(poseTarget)

    def positionTarget(self, target):
        # Given a target try and position arm end effector over it
        xyz = [0,0,0]
        xyz[0] = target.pose.position.x 
        xyz[1] = target.pose.position.y
        xyz[2] = target.pose.position.z
        # Send
        self.groupManipulator.set_position_target(xyz)

    def planPose(self):
        # Plan arm pose 
        plan = self.groupManipulator.plan()

    def executePlan(self):
        # Execute desired plan
        self.groupManipulator.go(wait=True)

    def transformPose(self, pose, posOffset, orienOffset, frame):
        # Transform pose using frame
        targetPose = PoseStamped()
        targetPose.header.frame_id = frame
        targetPose.pose.position.x = pose.position.x + posOffset[0]
        targetPose.pose.position.y = pose.position.y + posOffset[1]
        targetPose.pose.position.z = pose.position.z + posOffset[2]
        targetPose.pose.orientation.x = 0.0 + orienOffset[0]
        targetPose.pose.orientation.y = 0.0 + orienOffset[1]
        targetPose.pose.orientation.z = 0.0 + orienOffset[2]
        targetPose.pose.orientation.w = 0.0 + orienOffset[3]
        # Transform
        transformedPose = self.tf_listener.transformPose('world', targetPose)
        # Return
        return transformedPose

    def home(self):
        # Set arm position to 'home' position set in SRDF file
        rospy.loginfo("Set Arm: home")
        # Set
        self.groupManipulator.set_named_target('home')
        # Execute
        self.executePlan()

    def up(self):
        # Set arm position to 'up' position set in SRDF file
        rospy.loginfo("Set Arm: up")
        # Set
        self.groupManipulator.set_named_target('up')
        # Execute
        self.executePlan()

    def handlePart(self):
        # Set joint positions to a pose for handling part
        # Linear_Joint, Shoulder_Pan, Shoulder_Lift, Elbow, Wrist_1, Wrist_2, Wrist_3
        # [0, 3.14, -2.39, 2.39, 3.14, -1.51, 0.0]
        # Create a joint trajectory message and fill with values
        # Create names
        names = ['elbow_joint', 'linear_arm_actuator_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        # Create points
        point = JointTrajectoryPoint()
        point.positions = [2.39, 0.0, -2.39, 3.14, 3.14, -1.51, 0.0]
        point.time_from_start = rospy.Duration(3.0)
        # Execute
        self.op.jointTrajectory('Handle Part', names, point)

    def trayPose(self, frame):
        # Return the tray pose
        if frame == 'agv1_load_point_frame':
            # Return pose of agv on left
            (trans, rot) = self.tf_listener.lookupTransform('world', frame, rospy.Time(0))
        else:
            # Return pose of agv on right
            (trans, rot) = self.tf_listener.lookupTransform('world', frame, rospy.Time(0))
        # Create pose
        pose = PoseStamped()
        pose.header.frame_id = 'Tray Target'
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.x = rot[0]
        pose.pose.orientation.y = rot[1]
        pose.pose.orientation.z = rot[2]
        pose.pose.orientation.w = rot[3]
        # Return
        return pose
            

    def moveToTray(self, direction):
        # Move to either left or right tray
        # Linear_Joint, Shoulder_Pan, Shoulder_Lift, Elbow, Wrist_1, Wrist_2, Wrist_3
        # [2.1, 1.54, -2.39, 2.39, 3.14, -1.51, 0.0]
        # and for Rght
        # [-2.1, -1.54, -2.39, 2.39, 3.14, -1.51, 0.0]
        if direction == 'Left':
            # Move left
            # Create a joint trajectory message and fill with values
            # Create names
            names = ['elbow_joint', 'linear_arm_actuator_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            # Create points
            point = JointTrajectoryPoint()
            point.positions = [2.39, 2.1, -2.39, 1.54, 3.14, -1.51, 0.0]
            point.time_from_start = rospy.Duration(3.0)
            # Execute
            self.op.jointTrajectory('Move to left tray', names, point)
        else:
            # Move right
            # Create a joint trajectory message and fill with values
            # Create names
            names = ['elbow_joint', 'linear_arm_actuator_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            # Create points
            point = JointTrajectoryPoint()
            point.positions = [2.39, -2.1, -2.39, 4.62, 3.14, -1.51, 0.0]
            point.time_from_start = rospy.Duration(3.0)
            # Execute
            self.op.jointTrajectory('Move to right tray', names, point)

    def moveOverTray(self, direction):
        # Position arm over tray and in place to simply drop part
        # Linear_Joint, Shoulder_Pan, Shoulder_Lift, Elbow, Wrist_1, Wrist_2, Wrist_3
        # [2.10, 1.41, -0.51, 1.26, 3.90, -1.51, 0.0]
        if direction == 'Left':
            # Move left
            # Create a joint trajectory message and fill with values
            # Create names
            names = ['elbow_joint', 'linear_arm_actuator_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            # Create points
            point = JointTrajectoryPoint()
            point.positions = [1.26, 2.1, -0.51, 1.41, 3.9, -1.51, 0.0]
            point.time_from_start = rospy.Duration(2.0)
            # Execute
            self.op.jointTrajectory('Position over tray', names, point)
        else:
            # Move right
            # Create a joint trajectory message and fill with values
            # Create names
            names = ['elbow_joint', 'linear_arm_actuator_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            # Create points
            point = JointTrajectoryPoint()
            point.positions = [1.26, -2.1, -0.51, 4.62, 3.9, -1.51, 0.0]
            point.time_from_start = rospy.Duration(2.0)
            # Execute
            self.op.jointTrajectory('Position over tray', names, point)

    def moveOverInventory(self):
        # Position the arm to hover over the inventory
        # Create a joint trajectory message and fill with values
        # Create names
        names = ['elbow_joint', 'linear_arm_actuator_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        # Create points
        point = JointTrajectoryPoint()
        point.positions = [2.26, 0.0, -1.26, 3.27, 3.52, -1.51, 0.0]
        point.time_from_start = rospy.Duration(2.0)
        # Execute
        self.op.jointTrajectory('Position over tray', names, point)

    def armState(self):
        # Query arm state. In this instance it is either moving or still based upon velocities
        armState = self.op.armStateData
        totalVel = 0
        state = None
        # Check velocities
        desiredPoints = armState.desired
        # Get sum and divide by joint number
        for vel in desiredPoints.velocities:
            # Get absolute value
            totalVel += abs(vel)
        # Check
        if (totalVel / 7 > 0.05):
            # Robot arm is moving
            if self.pastArmState != 'Moving':
                # Print change
                rospy.loginfo("Arm State: Moving")
            state = 'Moving'
        else:
            # Robot arm is still
            if self.pastArmState != 'Still':
                # Print change
                rospy.loginfo("Arm State: Still")
            state = 'Still'
        # Set past arm state for loggin purposes
        self.pastArmState = state
        # Return state
        return state

    def eulerFromQuaternion(self, pose):
        # Get euler values from quaternion values
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        # Return
        return (roll, pitch, yaw)

    def quaternionFromEuler(self, pose):
        # Get quaternion values from euler values
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        # Return
        return (roll, pitch, yaw)

    def gripper(self, state):
        # Set vacumm gripper state
        self.op.vacuumGripper(state)

