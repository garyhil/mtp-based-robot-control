### Testing file
from datetime import datetime
import sys

import rospy
import moveit_commander

from mtppy.opcua_server_pea import OPCUAServerPEA
from mtppy.mtp_generator import MTPGenerator

from service.RobotServices import MoveService, HandService

def main():
    try:
        ### Defining a virtual PEA for the Franka Emika Robot
        robot = OPCUAServerPEA(endpoint='opc.tcp://127.0.0.1:4840/')

        ### Setting up ROS environment
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot')

        ### A simple MoveService, moving the EE
        move_service = MoveService(tag_name="Move-Service", tag_description='', fake_hw=True)
        hand_service = HandService(tag_name="Hand-Service", tag_description='', fake_hw=True)

        robot.add_service(move_service)
        robot.add_service(hand_service)

        # MTP File Generation
        # writer_info_dict = {
        #                     'WriterName': 'tud/plt', 'WriterID': 'tud/plt', 'WriterVendor': 'tud',
        #                     'WriterVendorURL': 'www.tud.de',
        #                     'WriterVersion': '1.0.0', 'WriterRelease': '', 'LastWritingDateTime': str(datetime.now()),
        #                     'WriterProjectTitle': 'tu/plt/mtp', 'WriterProjectID': ''
        #                     }
        # export_manifest_path = './manifest_files/robot_manifest.aml'
        # mtp_generator = MTPGenerator(writer_info_dict, export_manifest_path)
        robot.run_opcua_server()
    
    except:
        if robot is not None:
            robot.stop_opcua_server()
        sys.exit(0)

if __name__ == "__main__":
    main()

#move_service.op_src_mode.set_state_op_op(True)
#hand_service.op_src_mode.set_state_op_op(True)