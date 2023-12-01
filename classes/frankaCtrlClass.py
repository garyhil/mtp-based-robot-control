#!/usr/bin/python3
import rospy
import actionlib_msgs.msg
import geometry_msgs.msg
from moveit_msgs.srv import GetStateValidity

class FrankaControlBaseClass:
    def __init__(self, robot, group, scene, pose, moveSettings, name):
        rospy.loginfo('Waiting for move_group/status')
        rospy.wait_for_message('move_group/status', actionlib_msgs.msg.GoalStatusArray)
        
        self.robot = robot
        self.group = group
        self.scene = scene
        self.pose = pose
        self.moveSettings = moveSettings
        self.name = name

        self.plannedPath = None

        rospy.wait_for_service('/check_state_validity')
        self.check_collision = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
    
    def update_pose(self, new_pose):
        self.pose = new_pose

class MoveControl(FrankaControlBaseClass):
    def __init__(self, robot, group, scene, pose, moveSettings, name):
        super().__init__(robot=robot, group=group, scene=scene, pose=pose,moveSettings=moveSettings, name=name)

    def reach_pose_via_joints(self):
        ''' Move the robot to the desired pose via joint values.

       Returns:
           bool: True if the robot successfully reaches the pose, False otherwise.
       '''
        self.group.set_joint_value_target(self.pose[2])
            
        success = False 
        while(success==False):
            self.planned_path = self.group.plan()
            
            success = self.group.execute(self.planned_path[1],wait=True)

        rospy.loginfo(self.name + "reached!")  
        return success
    
    def reach_pose_via_posquat(self):
        ''' Move the robot to the desired pose via Position/Quaternion values.

       Returns:
           bool: True if the robot successfully reaches the pose, False otherwise.
       '''
        posquat = geometry_msgs.msg.Pose()
        posquat.position = geometry_msgs.msg.Point(x=self.pose[0][0],y=self.pose[0][1], z=self.pose[0][2])
        posquat.orientation = geometry_msgs.msg.Quaternion(w=self.pose[1][0],x=self.pose[1][1],y=self.pose[1][2],z=self.pose[1][3])
        self.group.set_pose_target(posquat)

        success = False 
        while(success==False):
            self.planned_path = self.group.plan()
            
            success = self.group.execute(self.planned_path[1],wait=True)

        rospy.loginfo(self.name + "reached!")  
        return success

class HandControl(FrankaControlBaseClass):
    def __init__(self, robot, group, scene, pose, moveSettings, name):
        super().__init__(robot=robot, group=group, scene=scene, pose=pose,moveSettings=moveSettings, name=name)

    def grasp(self):
        ''' Close the Gripper to desired distance.

       Returns:
           bool: True if the gripper can successfully close the fingers. False otherwise.
       '''
        self.group.set_joint_value_target(self.pose)
            
        success = False 
        while(success==False):
            self.planned_path = self.group.plan()
            
            success = self.group.execute(self.planned_path[1],wait=True)

        rospy.loginfo("Gripper closed!")  
        return success
    
    def open(self):
        ''' Open the Gripper to desired distance.

       Returns:
           bool: True if the gripper can successfully open the fingers. False otherwise.
       '''
        self.group.set_joint_value_target(self.pose)
            
        success = False 
        while(success==False):
            self.planned_path = self.group.plan()
            
            success = self.group.execute(self.planned_path[1],wait=True)

        rospy.loginfo("Gripper opened!")  
        return success
    