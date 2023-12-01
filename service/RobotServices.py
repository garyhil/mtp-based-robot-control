### Testing file
import sys
### ROS specific Libs
import rospy
import moveit_commander
import controller_manager_msgs.srv
### MTP for Python specific Libs
from mtppy.service import Service
from mtppy.procedure import Procedure
from mtppy.operation_elements import AnaServParam
### Own Libs
import robot_positions.positions as positions
from classes.frankaCtrlClass import MoveControl, HandControl


class MoveService(Service):
    def __init__(self, tag_name: str, tag_description: str, fake_hw:bool):
        super().__init__(tag_name, tag_description)
        if fake_hw is False:
            rospy.loginfo('Waiting for controller_manager/load_controller')
            load_controller = rospy.ServiceProxy('controller_manager/load_controller',controller_manager_msgs.srv.LoadController)
            load_controller.wait_for_service()

            for controller_name in ['position_joint_trajectory_controller','effort_joint_trajectory_controller']:
                if not load_controller(controller_name):
                    rospy.logerr('Could not load {}', controller_name)
                    sys.exit(1)

        rospy.loginfo('Loaded controllers')
        group = moveit_commander.MoveGroupCommander('panda_manipulator')
        robot = moveit_commander.RobotCommander('robot_description')
        scene = moveit_commander.PlanningSceneInterface(synchronous = True)
        relMoveVel = 0.7
        relMoveAcc = 0.2
        moveSettings= [relMoveVel, relMoveAcc]
        scene.clear()
        group.set_max_velocity_scaling_factor(0.4)
        group.set_max_acceleration_scaling_factor(0.2)
        group.set_planner_id("RRTConnect")
        group.set_planning_time(30)
        group.set_num_planning_attempts(45)
        self.defaultpose = [positions.pos1, positions.rot1, positions.joints1]

        #createCollissionObject(id='table',form=objectType.BOX,dimensions=[0.8, 1.0, 0.6], scene=scene, pos=[0.0,0.0,-0.297])
        self.movetask = MoveControl(robot=robot,group=group,scene=scene,pose=self.defaultpose,moveSettings=moveSettings,name="target_1")

        ### Procedure using joint values (j1 - j7) to move to a specific pose relative to the robots base
        movejoints_rel2base_procedure = Procedure(procedure_id=1, tag_name="MoveViaJoints",is_self_completing=True)
        ### Procedure using Position (x,y,z) and Quaternion(qx,qy,qz,qw) to move to a specific pose relative to the robots base
        moveposquat_rel2base_procedure = Procedure(procedure_id=2, tag_name="MoveViaPosQuat",is_self_completing=True)

        ### Required procedure parameters
        moveposquat_rel2base_procedure.add_procedure_parameter(AnaServParam(tag_name="X-Coordinate",  tag_description='', v_min=-0.845, v_max=0.845))
        moveposquat_rel2base_procedure.add_procedure_parameter(AnaServParam(tag_name="Y-Coordinate",  tag_description='', v_min=-0.350, v_max=1.180))
        moveposquat_rel2base_procedure.add_procedure_parameter(AnaServParam(tag_name="Z-Coordinate",  tag_description='', v_min=-0.845, v_max=0.845))
        moveposquat_rel2base_procedure.add_procedure_parameter(AnaServParam(tag_name="QX-Quaternion", tag_description='', v_min=-1000, v_max=1000))
        moveposquat_rel2base_procedure.add_procedure_parameter(AnaServParam(tag_name="QY-Quaternion", tag_description='', v_min=-1000, v_max=1000))
        moveposquat_rel2base_procedure.add_procedure_parameter(AnaServParam(tag_name="QZ-Quaternion", tag_description='', v_min=-1000, v_max=1000))
        moveposquat_rel2base_procedure.add_procedure_parameter(AnaServParam(tag_name="QW-Quaternion", tag_description='', v_min=-1000, v_max=1000))
        movejoints_rel2base_procedure.add_procedure_parameter(AnaServParam(tag_name="Joint1", tag_description='', v_min=-2.8963, v_max=2.8963))
        movejoints_rel2base_procedure.add_procedure_parameter(AnaServParam(tag_name="Joint2", tag_description='', v_min=-1.7618, v_max=1.7618))
        movejoints_rel2base_procedure.add_procedure_parameter(AnaServParam(tag_name="Joint3", tag_description='', v_min=-2.8963, v_max=2.8963))
        movejoints_rel2base_procedure.add_procedure_parameter(AnaServParam(tag_name="Joint4", tag_description='', v_min=-3.0708, v_max=-0.0688))
        movejoints_rel2base_procedure.add_procedure_parameter(AnaServParam(tag_name="Joint5", tag_description='', v_min=-2.8963, v_max=2.8963))
        movejoints_rel2base_procedure.add_procedure_parameter(AnaServParam(tag_name="Joint6", tag_description='', v_min=-0.0165, v_max=3.7515))
        movejoints_rel2base_procedure.add_procedure_parameter(AnaServParam(tag_name="Joint7", tag_description='', v_min=-2.8963, v_max=2.8963))

        self.add_procedure(movejoints_rel2base_procedure)
        self.add_procedure(moveposquat_rel2base_procedure)
        

    def idle(self):
        """
        Idle state.
        :return:
        """
        if self.procedure_control.get_procedure_cur() != 0:
            print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Idle State!")
        
    def starting(self):
        """
        Starting state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Starting State!")

        if self.procedure_control.get_procedure_cur() == 1:
            self.procedures[1].procedure_parameters['Joint1'].set_v_out()
            self.procedures[1].procedure_parameters['Joint2'].set_v_out()
            self.procedures[1].procedure_parameters['Joint3'].set_v_out()
            self.procedures[1].procedure_parameters['Joint4'].set_v_out()
            self.procedures[1].procedure_parameters['Joint5'].set_v_out()
            self.procedures[1].procedure_parameters['Joint6'].set_v_out()
            self.procedures[1].procedure_parameters['Joint7'].set_v_out()
        elif self.procedure_control.get_procedure_cur() == 2:
            self.procedures[2].procedure_parameters['X-Coordinate'].set_v_out()
            self.procedures[2].procedure_parameters['Y-Coordinate'].set_v_out()
            self.procedures[2].procedure_parameters['Z-Coordinate'].set_v_out()
            self.procedures[2].procedure_parameters['QX-Quaternion'].set_v_out()
            self.procedures[2].procedure_parameters['QY-Quaternion'].set_v_out()
            self.procedures[2].procedure_parameters['QZ-Quaternion'].set_v_out()
            self.procedures[2].procedure_parameters['QW-Quaternion'].set_v_out()
        else:
            print("no valid Procedure ID")

        new_pos = []
        new_rot = []
        new_joints = []
        
        new_pos.append(self.procedures[2].procedure_parameters['X-Coordinate'].get_v_out())
        new_pos.append(self.procedures[2].procedure_parameters['Y-Coordinate'].get_v_out())
        new_pos.append(self.procedures[2].procedure_parameters['Z-Coordinate'].get_v_out())

        new_rot.append(self.procedures[2].procedure_parameters['QW-Quaternion'].get_v_out())
        new_rot.append(self.procedures[2].procedure_parameters['QX-Quaternion'].get_v_out())
        new_rot.append(self.procedures[2].procedure_parameters['QY-Quaternion'].get_v_out())
        new_rot.append(self.procedures[2].procedure_parameters['QZ-Quaternion'].get_v_out())
        
        new_joints.append(self.procedures[1].procedure_parameters['Joint1'].get_v_out())
        new_joints.append(self.procedures[1].procedure_parameters['Joint2'].get_v_out())
        new_joints.append(self.procedures[1].procedure_parameters['Joint3'].get_v_out())
        new_joints.append(self.procedures[1].procedure_parameters['Joint4'].get_v_out())
        new_joints.append(self.procedures[1].procedure_parameters['Joint5'].get_v_out())
        new_joints.append(self.procedures[1].procedure_parameters['Joint6'].get_v_out())
        new_joints.append(self.procedures[1].procedure_parameters['Joint7'].get_v_out())

        new_pose = [new_pos, new_rot, new_joints]

        self.movetask.update_pose(new_pose=new_pose)
        self.state_change()
        return  
    
    def execute(self):
        """
        Execute state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Execute State!")
        if self.procedure_control.get_procedure_cur() == 1:
            self.movetask.reach_pose_via_joints()
            self.state_change()
        elif self.procedure_control.get_procedure_cur() == 2:
            self.movetask.reach_pose_via_posquat()
            self.state_change()
        else:
            print("no valid Procedure ID")

        return
        
    def completing(self):
        """
        Completing state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Completing State!")
        self.state_change()
        return
        
    def completed(self):
        """
        Completed state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Completed State!")
        return
        
    def pausing(self):
        """
        Pausing state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Pausing State!")
        self.state_change()
        return
          
    def paused(self):
        """
        Paused state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Paused State!")
        return
          
    def resuming(self):
        """
        Resuming state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Resuming State!")
        self.state_change()
        return
        
    def holding(self):
        """
        Holding state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Holding State!")
        self.state_change()
        return
        
    def held(self):
        """
        Held state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Held State!")
        return
        
    def unholding(self):
        """
        Unholding state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Unholding State!")
        self.state_change()
        return
        
    def stopping(self):
        """
        Stopping state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Stopping State!")
        self.state_change()
        return
        
    def stopped(self):
        """
        Stopped state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Stopped State!")
        self.state_change()
        return
        
    def aborting(self):
        """
        Aborting state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Aborting State!")
        self.state_change()
        return
        
    def aborted(self):
        """
        Aborted state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Aborted State!")
        return
        
    def resetting(self):
        """
        Resetting state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Resetting State!")
        self.state_change()
        return
    
class HandService(Service):
    def __init__(self, tag_name: str, tag_description: str, fake_hw:bool):
        super().__init__(tag_name, tag_description)
        if fake_hw is False:
            rospy.loginfo('Waiting for controller_manager/load_controller')
            load_controller = rospy.ServiceProxy('controller_manager/load_controller',controller_manager_msgs.srv.LoadController)
            load_controller.wait_for_service()

            for controller_name in ['position_joint_trajectory_controller','effort_joint_trajectory_controller']:
                if not load_controller(controller_name):
                    rospy.logerr('Could not load {}', controller_name)
                    sys.exit(1)

        rospy.loginfo('Loaded controllers')
        group = moveit_commander.MoveGroupCommander('panda_hand')
        robot = moveit_commander.RobotCommander('robot_description')
        scene = moveit_commander.PlanningSceneInterface(synchronous = True)
        relMoveVel = 0.7
        relMoveAcc = 0.2
        moveSettings= [relMoveVel, relMoveAcc]
        scene.clear()
        group.set_max_velocity_scaling_factor(0.4)
        group.set_max_acceleration_scaling_factor(0.2)
        group.set_planner_id("RRTConnect")
        group.set_planning_time(30)
        group.set_num_planning_attempts(45)
        self.defaultpose = [positions.width1/2, positions.width1/2]
        self.movetask = HandControl(robot=robot,group=group,scene=scene,pose=self.defaultpose,moveSettings=moveSettings,name="target_1")

        ## Procedure Definition
        openProcedure = Procedure(procedure_id=1, tag_name="OpenGripper", tag_description='', is_self_completing=True)
        closeProcedure = Procedure(procedure_id=2, tag_name="CloseGripper", tag_description='', is_self_completing=True)
        ## Procedure Parameters
        openProcedure.add_procedure_parameter(AnaServParam(tag_name='OpeningWidth', tag_description='', v_min=0.001, v_max=0.079, v_unit=1010))
        closeProcedure.add_procedure_parameter(AnaServParam(tag_name='ClosingWidth', tag_description='', v_min=0.001, v_max=0.079, v_unit=1010))

        ## Add Procedures to Service
        self.add_procedure(openProcedure)
        self.add_procedure(closeProcedure)


    def idle(self):
        """
        Idle state.
        :return:
        """
        if self.procedure_control.get_procedure_cur() != 0:
            print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Idle State!")
        
    def starting(self):
        """
        Starting state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Starting State!")
        print('Applying procedure parameters...')
        if self.procedure_control.get_procedure_cur() == 1:
            anaserv = self.procedures[1].procedure_parameters['OpeningWidth']
            anaserv.set_v_out()
        elif self.procedure_control.get_procedure_cur() == 2:
            anaserv = self.procedures[2].procedure_parameters['ClosingWidth']
            anaserv.set_v_out()

        new_pose = []
        if self.procedure_control.get_procedure_cur() == 1:
            width = self.procedures[1].procedure_parameters['OpeningWidth'].get_v_out()
            new_pose.append((width)/2)
            new_pose.append((width)/2)
        elif self.procedure_control.get_procedure_cur() == 2:
            width = self.procedures[2].procedure_parameters['ClosingWidth'].get_v_out()
            new_pose.append((width)/2)
            new_pose.append((width)/2)
        else:
            print("no valid Procedure ID")

        self.movetask.update_pose(new_pose=new_pose)
        self.state_change()
        return  
    
    def execute(self):
        """
        Execute state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Execute State!")
        if self.procedure_control.get_procedure_cur() == 1:
            self.movetask.open()
            self.state_change()
        elif self.procedure_control.get_procedure_cur() == 2:
            self.movetask.grasp()
            self.state_change()
        else:
            print("no valid Procedure ID")
        return
        
    def completing(self):
        """
        Completing state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Completing State!")
        self.state_change()
        return
        
    def completed(self):
        """
        Completed state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Completed State!")
        return
        
    def pausing(self):
        """
        Pausing state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Pausing State!")
        self.state_change()
        return
          
    def paused(self):
        """
        Paused state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Paused State!")
        return
          
    def resuming(self):
        """
        Resuming state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Resuming State!")
        self.state_change()
        return
        
    def holding(self):
        """
        Holding state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Holding State!")
        self.state_change()
        return
        
    def held(self):
        """
        Held state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Held State!")
        return
        
    def unholding(self):
        """
        Unholding state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Unholding State!")
        self.state_change()
        return
        
    def stopping(self):
        """
        Stopping state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Stopping State!")
        self.state_change()
        return
        
    def stopped(self):
        """
        Stopped state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Stopped State!")
        self.state_change()
        return
        
    def aborting(self):
        """
        Aborting state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Aborting State!")
        self.state_change()
        return
        
    def aborted(self):
        """
        Aborted state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Aborted State!")
        return
        
    def resetting(self):
        """
        Resetting state.
        :return:
        """
        print(f"Service: {self.tag_name} with Procedure: {self.procedures[self.procedure_control.get_procedure_cur()].tag_name} in Resetting State!")
        self.state_change()
        return