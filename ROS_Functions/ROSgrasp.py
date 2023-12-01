#!/usr/bin/python3
import sys
import rospy
import moveit_commander
import controller_manager_msgs.srv
from moveit_msgs.msg import CollisionObject
import robot_positions.positions as positions
from classes.frankaCtrlClass import HandControl

 
def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('hand_node')
    rospy.loginfo('Waiting for controller_manager/load_controller')
    #load_controller = rospy.ServiceProxy('controller_manager/load_controller',controller_manager_msgs.srv.LoadController)
    #load_controller.wait_for_service()
    #
    #for controller_name in ['position_joint_trajectory_controller','effort_joint_trajectory_controller']:
    #    if not load_controller(controller_name):
    #        rospy.logerr('Could not load {}', controller_name)
    #        sys.exit(1)
    
    rospy.loginfo('Loaded controllers')

    group = moveit_commander.MoveGroupCommander('panda_hand')
    robot = moveit_commander.RobotCommander('robot_description')
    scene = moveit_commander.PlanningSceneInterface(synchronous = True)
    pose1 = [0.02, 0.02]
    pose2 = [0.03, 0.03]
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
    graspTask = HandControl(robot=robot,group=group,scene=scene,pose=pose1,moveSettings=moveSettings,name="target_1")
    releasetTask = HandControl(robot=robot,group=group,scene=scene,pose=pose2,moveSettings=moveSettings,name="target_2")

    graspTask.grasp()
    releasetTask.open()

if __name__ == '__main__':
    main()