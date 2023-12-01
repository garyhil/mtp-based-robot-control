#!/usr/bin/python3
import sys
import rospy
import moveit_commander
import controller_manager_msgs.srv
from moveit_msgs.msg import CollisionObject
import robot_positions.positions as positions
from classes.frankaCtrlClass import Move2Pose

 
def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_place')
    rospy.loginfo('Waiting for controller_manager/load_controller')
    #load_controller = rospy.ServiceProxy('controller_manager/load_controller',controller_manager_msgs.srv.LoadController)
    #load_controller.wait_for_service()
    #
    #for controller_name in ['position_joint_trajectory_controller','effort_joint_trajectory_controller']:
    #    if not load_controller(controller_name):
    #        rospy.logerr('Could not load {}', controller_name)
    #        sys.exit(1)
    
    rospy.loginfo('Loaded controllers')

    group = moveit_commander.MoveGroupCommander('panda_manipulator')
    robot = moveit_commander.RobotCommander('robot_description')
    scene = moveit_commander.PlanningSceneInterface(synchronous = True)
    pose1 = [positions.pos1, positions.rot1, positions.joints1]
    pose2 = [positions.pos2, positions.rot2, positions.joints2]
    pose3 = [positions.pos3, positions.rot3, positions.joints3]
    relMoveVel = 0.7
    relMoveAcc = 0.2
    moveSettings= [relMoveVel, relMoveAcc]
    scene.clear()
    group.set_max_velocity_scaling_factor(0.4)
    group.set_max_acceleration_scaling_factor(0.2)
    group.set_planner_id("RRTConnect")
    group.set_planning_time(30)
    group.set_num_planning_attempts(45)
    
    #createCollissionObject(id='table',form=objectType.BOX,dimensions=[0.8, 1, 0.6], scene=scene, pos=[0.0,0.0,-0.297])
    movetask1 = Move2Pose(robot=robot,group=group,scene=scene,pose=pose1,moveSettings=moveSettings,name="target_1")
    movetask2 = Move2Pose(robot=robot,group=group,scene=scene,pose=pose2,moveSettings=moveSettings,name="target_2")
    movetask3 = Move2Pose(robot=robot,group=group,scene=scene,pose=pose3,moveSettings=moveSettings,name="target_3")

    movetask1.reach_pose_via_posquat()
    movetask2.reach_pose_via_posquat()
    movetask3.reach_pose_via_joints()

if __name__ == '__main__':
    main()