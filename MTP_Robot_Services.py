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
        # MTP File Generation
        writer_info_dict = {
                             'WriterName': 'PforzheimUniversity/Engineerium', 'WriterID': 'PforzheimUniversity/Engineerium', 'WriterVendor': 'PforzheimUniversity',
                             'WriterVendorURL': 'www.hs-pforzheim.de',
                             'WriterVersion': '1.0.0', 'WriterRelease': '', 'LastWritingDateTime': str(datetime.now()),
                             'WriterProjectTitle': 'PforzheimUniversity/Engineerium/mtp4robots', 'WriterProjectID': ''
                             }
        export_manifest_path = './MTP-files/robots_manifest.aml'
        manifest_template_path = './MTP-files/manifest_template.xml'  
        mtp_generator = MTPGenerator(writer_info_dict, export_manifest_path, manifest_template_path=manifest_template_path)

        ### Defining a virtual PEA for the Franka Emika Robot
        robot = OPCUAServerPEA(mtp_generator=mtp_generator,endpoint='opc.tcp://127.0.0.1:4840/')

        ### Setting up ROS environment
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot')

        ### A simple MoveService, moving the EE
        move_service = MoveService(tag_name="Move-Service", tag_description='', fake_hw=True)
        hand_service = HandService(tag_name="Hand-Service", tag_description='', fake_hw=True)

        robot.add_service(move_service)
        robot.add_service(hand_service)

        robot.run_opcua_server()

    
    except:
        if robot is not None:
            robot.stop_opcua_server()
        sys.exit(0)

if __name__ == "__main__":
    main()

#move_service.op_src_mode.set_state_op_op(True)
#hand_service.op_src_mode.set_state_op_op(True)