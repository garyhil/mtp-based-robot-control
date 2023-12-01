#!/usr/bin/env python

import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion

def add_box_to_planning_scene(robot:RobotCommander(), scene:PlanningSceneInterface(), boxName:str, boxSize:list(), boxPose:Pose()):
    # Initialize the ROS node
    rospy.init_node('add_box_to_planning_scene', anonymous=True)

    # Initialize the MoveIt! commander

    # Define the frame in which the box will be placed
    reference_frame = robot.get_planning_frame()

    # Define the pose of the box (position and orientation)
    boxPose = PoseStamped()
    boxPose.header.frame_id = reference_frame

    # Add the box to the planning scene
    scene.add_box(boxName, boxPose, size=boxSize)

    # Sleep to allow time for the planning scene to update
    rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        robot=RobotCommander()
        scene = PlanningSceneInterface()
        boxName = "TestBox"
        boxSize = (0.1,0.1,0.1)
        boxPose = Pose(position=Point(x=1.0,y=1.0,z=1.0), orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=0.0))
        add_box_to_planning_scene(robot=robot, scene=scene, boxName=boxName, boxSize=boxSize, boxPose=boxPose)
    except rospy.ROSInterruptException:
        pass
