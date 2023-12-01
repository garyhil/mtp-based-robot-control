import time

from opcua import Client
from mtppy.command_codes import CommandCodes
from mtppy.state_codes import StateCodes
import robot_positions.positions as robot_values

# Replace 'opc.tcp://localhost:4840/freeopcua/server/' with the address of your OPC UA server
server_url = "opc.tcp://127.0.0.1:4840/"

# Create a client instance
client = Client(server_url)
commandCodes = CommandCodes()
stateCodes = StateCodes()


def get_service_nodes_as_dict(node, node_dict=None):
    """
    Recursively explore nodes in the OPC UA server's address space and create a nested dictionary
    representing the hierarchical structure of the nodes. Each node is represented by a dictionary
    containing the NodeID and child nodes.
    """
    if node_dict is None:
        node_dict = {}

    for child in node.get_children():
        child_name = child.get_browse_name().Name
        child_node_id = child.nodeid.to_string()

        # Recursively explore child nodes
        child_dict = {"NodeID": child_node_id}
        get_service_nodes_as_dict(child, node_dict=child_dict)

        # Add child_dict to node_dict
        node_dict[child_name] = child_dict

    return node_dict

def opening_Hand(services:dict, width:float):
    print("Next we set the Hand-Service to Operator Mode!")
    input("Press Enter to continue!")
    node = client.get_node(services['Hand-Service']['op_src_mode']['StateOpOp']['NodeID'])
    node.set_value(True)
    
    print("Next we set the requested procedure of Hand-Service to 'open' (id = 1)")
    input("Press Enter to continue!")
    node = client.get_node(services['Hand-Service']['procedure_control']['ProcedureOp']['NodeID'])
    node.set_value(1)

    print("Next we set the Procedure Parameter to Operator Mode!")
    input("Press Enter to continue!")
    node = client.get_node(services['Hand-Service']['procedures']['OpenGripper']['procedure_parameters']['OpeningWidth']['op_src_mode']['StateOpOp']['NodeID'])
    node.set_value(True)

    print("Next we set the Opening Width requested")
    input("Press Enter to continue!")
    node = client.get_node(services['Hand-Service']['procedures']['OpenGripper']['procedure_parameters']['OpeningWidth']['VOp']['NodeID'])
    node.set_value(width)

    print("Next we start the Service")
    input("Press Enter to continue!")
    node = client.get_node(services['Hand-Service']['state_machine']['CommandOp']['NodeID'])
    node.set_value(commandCodes.start)

    print("We wait till the Service is completed and reset the Service to idle-Mode")
    node = client.get_node(services['Hand-Service']['state_machine']['StateCur']['NodeID'])
    while node.get_value() != stateCodes.completed:
        time.sleep(0.1)
    node = client.get_node(services['Hand-Service']['state_machine']['CommandOp']['NodeID'])
    node.set_value(commandCodes.reset)

def closing_Hand(services:dict, width:float):
    print("Next we set the Hand-Service to Operator Mode!")
    input("Press Enter to continue!")
    node = client.get_node(services['Hand-Service']['op_src_mode']['StateOpOp']['NodeID'])
    node.set_value(True)
    
    print("Next we set the requested procedure of Hand-Service to 'close' (id = 2)")
    input("Press Enter to continue!")
    node = client.get_node(services['Hand-Service']['procedure_control']['ProcedureOp']['NodeID'])
    node.set_value(2)

    print("Next we set the Procedure Parameter to Operator Mode!")
    input("Press Enter to continue!")
    node = client.get_node(services['Hand-Service']['procedures']['CloseGripper']['procedure_parameters']['ClosingWidth']['op_src_mode']['StateOpOp']['NodeID'])
    node.set_value(True)

    print("Next we set the Closing Width requested")
    input("Press Enter to continue!")
    node = client.get_node(services['Hand-Service']['procedures']['CloseGripper']['procedure_parameters']['ClosingWidth']['VOp']['NodeID'])
    node.set_value(width)

    print("Next we start the Service")
    input("Press Enter to continue!")
    node = client.get_node(services['Hand-Service']['state_machine']['CommandOp']['NodeID'])
    node.set_value(commandCodes.start)

    print("We wait till the Service is completed and reset the Service to idle-Mode")
    node = client.get_node(services['Hand-Service']['state_machine']['StateCur']['NodeID'])
    while node.get_value() != stateCodes.completed:
        time.sleep(0.1)
    node = client.get_node(services['Hand-Service']['state_machine']['CommandOp']['NodeID'])
    node.set_value(commandCodes.reset)

def moving_robot_via_joints(services:dict, joint_values:list):
    
    print("Next we set the Move-Service to Operator Mode!")
    input("Press Enter to continue!")
    node = client.get_node(services['Move-Service']['op_src_mode']['StateOpOp']['NodeID'])
    node.set_value(True)
    
    
    print("Next we set the requested procedure of Move-Service to move via joint values (id = 1)")
    input("Press Enter to continue!")
    node = client.get_node(services['Move-Service']['procedure_control']['ProcedureOp']['NodeID'])
    node.set_value(1)

    print("Next we set the Procedure Parameters to Operator Mode!")
    input("Press Enter to continue!")
    node = client.get_node(services['Move-Service']['procedures']['MoveViaJoints']['procedure_parameters']['Joint1']['op_src_mode']['StateOpOp']['NodeID'])
    node.set_value(True)
    node = client.get_node(services['Move-Service']['procedures']['MoveViaJoints']['procedure_parameters']['Joint2']['op_src_mode']['StateOpOp']['NodeID'])
    node.set_value(True)
    node = client.get_node(services['Move-Service']['procedures']['MoveViaJoints']['procedure_parameters']['Joint3']['op_src_mode']['StateOpOp']['NodeID'])
    node.set_value(True)
    node = client.get_node(services['Move-Service']['procedures']['MoveViaJoints']['procedure_parameters']['Joint4']['op_src_mode']['StateOpOp']['NodeID'])
    node.set_value(True)
    node = client.get_node(services['Move-Service']['procedures']['MoveViaJoints']['procedure_parameters']['Joint5']['op_src_mode']['StateOpOp']['NodeID'])
    node.set_value(True)
    node = client.get_node(services['Move-Service']['procedures']['MoveViaJoints']['procedure_parameters']['Joint6']['op_src_mode']['StateOpOp']['NodeID'])
    node.set_value(True)
    node = client.get_node(services['Move-Service']['procedures']['MoveViaJoints']['procedure_parameters']['Joint7']['op_src_mode']['StateOpOp']['NodeID'])
    node.set_value(True)

    print("Next we set the desired joint values")
    input("Press Enter to continue!")
    node = client.get_node(services['Move-Service']['procedures']['MoveViaJoints']['procedure_parameters']['Joint1']['VOp']['NodeID'])
    node.set_value(joint_values[0])
    node = client.get_node(services['Move-Service']['procedures']['MoveViaJoints']['procedure_parameters']['Joint2']['VOp']['NodeID'])
    node.set_value(joint_values[1])
    node = client.get_node(services['Move-Service']['procedures']['MoveViaJoints']['procedure_parameters']['Joint3']['VOp']['NodeID'])
    node.set_value(joint_values[2])
    node = client.get_node(services['Move-Service']['procedures']['MoveViaJoints']['procedure_parameters']['Joint4']['VOp']['NodeID'])
    node.set_value(joint_values[3])
    node = client.get_node(services['Move-Service']['procedures']['MoveViaJoints']['procedure_parameters']['Joint5']['VOp']['NodeID'])
    node.set_value(joint_values[4])
    node = client.get_node(services['Move-Service']['procedures']['MoveViaJoints']['procedure_parameters']['Joint6']['VOp']['NodeID'])
    node.set_value(joint_values[5])
    node = client.get_node(services['Move-Service']['procedures']['MoveViaJoints']['procedure_parameters']['Joint7']['VOp']['NodeID'])
    node.set_value(joint_values[6])

    print("Next we start the Service")
    input("Press Enter to continue!")
    node = client.get_node(services['Move-Service']['state_machine']['CommandOp']['NodeID'])
    node.set_value(commandCodes.start)

    print("We wait till the Service is completed and reset the Service to idle-Mode")
    node = client.get_node(services['Move-Service']['state_machine']['StateCur']['NodeID'])
    while node.get_value() != stateCodes.completed:
        time.sleep(0.1)
    node = client.get_node(services['Move-Service']['state_machine']['CommandOp']['NodeID'])
    node.set_value(commandCodes.reset)

def moving_robot_via_posquat(services:dict, pos:list, quat:list):
    
    print("Next we set the Move-Service to Operator Mode!")
    input("Press Enter to continue!")
    node = client.get_node(services['Move-Service']['op_src_mode']['StateOpOp']['NodeID'])
    node.set_value(True)
    
    
    print("Next we set the requested procedure of Move-Service to move via posquat values (id = 2)")
    input("Press Enter to continue!")
    node = client.get_node(services['Move-Service']['procedure_control']['ProcedureOp']['NodeID'])
    node.set_value(2)

    print("Next we set the Procedure Parameters to Operator Mode!")
    input("Press Enter to continue!")
    node = client.get_node(services['Move-Service']['procedures']['MoveViaPosQuat']['procedure_parameters']['X-Coordinate']['op_src_mode']['StateOpOp']['NodeID'])
    node.set_value(True)
    node = client.get_node(services['Move-Service']['procedures']['MoveViaPosQuat']['procedure_parameters']['Y-Coordinate']['op_src_mode']['StateOpOp']['NodeID'])
    node.set_value(True)
    node = client.get_node(services['Move-Service']['procedures']['MoveViaPosQuat']['procedure_parameters']['Z-Coordinate']['op_src_mode']['StateOpOp']['NodeID'])
    node.set_value(True)
    node = client.get_node(services['Move-Service']['procedures']['MoveViaPosQuat']['procedure_parameters']['QW-Quaternion']['op_src_mode']['StateOpOp']['NodeID'])
    node.set_value(True)
    node = client.get_node(services['Move-Service']['procedures']['MoveViaPosQuat']['procedure_parameters']['QX-Quaternion']['op_src_mode']['StateOpOp']['NodeID'])
    node.set_value(True)
    node = client.get_node(services['Move-Service']['procedures']['MoveViaPosQuat']['procedure_parameters']['QY-Quaternion']['op_src_mode']['StateOpOp']['NodeID'])
    node.set_value(True)
    node = client.get_node(services['Move-Service']['procedures']['MoveViaPosQuat']['procedure_parameters']['QZ-Quaternion']['op_src_mode']['StateOpOp']['NodeID'])
    node.set_value(True)

    print("Next we set the desired joint values")
    input("Press Enter to continue!")
    node = client.get_node(services['Move-Service']['procedures']['MoveViaPosQuat']['procedure_parameters']['X-Coordinate']['VOp']['NodeID'])
    node.set_value(pos[0])
    node = client.get_node(services['Move-Service']['procedures']['MoveViaPosQuat']['procedure_parameters']['Y-Coordinate']['VOp']['NodeID'])
    node.set_value(pos[1])
    node = client.get_node(services['Move-Service']['procedures']['MoveViaPosQuat']['procedure_parameters']['Z-Coordinate']['VOp']['NodeID'])
    node.set_value(pos[2])
    node = client.get_node(services['Move-Service']['procedures']['MoveViaPosQuat']['procedure_parameters']['QW-Quaternion']['VOp']['NodeID'])
    node.set_value(quat[0])
    node = client.get_node(services['Move-Service']['procedures']['MoveViaPosQuat']['procedure_parameters']['QX-Quaternion']['VOp']['NodeID'])
    node.set_value(quat[1])
    node = client.get_node(services['Move-Service']['procedures']['MoveViaPosQuat']['procedure_parameters']['QY-Quaternion']['VOp']['NodeID'])
    node.set_value(quat[2])
    node = client.get_node(services['Move-Service']['procedures']['MoveViaPosQuat']['procedure_parameters']['QZ-Quaternion']['VOp']['NodeID'])
    node.set_value(quat[3])

    print("Next we start the Service")
    input("Press Enter to continue!")
    node = client.get_node(services['Move-Service']['state_machine']['CommandOp']['NodeID'])
    node.set_value(commandCodes.start)

    print("We wait till the Service is completed and reset the Service to idle-Mode")
    node = client.get_node(services['Move-Service']['state_machine']['StateCur']['NodeID'])
    while node.get_value() != stateCodes.completed:
        time.sleep(0.1)
    node = client.get_node(services['Move-Service']['state_machine']['CommandOp']['NodeID'])
    node.set_value(commandCodes.reset)

def main():
    try:
        # Connect to the server
        client.connect()

        # Browse the server's objects
        print("Objects nodeids:")
        for obj in client.get_objects_node().get_children():
            print(obj)

        # Get the services of the module as nested dict
        services_id = "ns=3;s=services"
        services = get_service_nodes_as_dict(client.get_node(services_id))
        
        # Running all commands to prepare and run the Hand-Service with opening procedure
        opening_Hand(services, width=robot_values.width0)

        # Running all commands to prepare and run the Move-Service with joint based procedure
        moving_robot_via_joints(services, robot_values.joints1)

        # Running all commands to prepare and run the Hand-Service with closing procedure
        closing_Hand(services, robot_values.width1)

        # Running all commands to prepare and run the Move-Service with joint based procedure
        moving_robot_via_posquat(services, robot_values.pos3, robot_values.rot3)

        # Running all commands to prepare and run the Hand-Service with opening procedure
        opening_Hand(services, width=robot_values.width2)

        # Running all commands to prepare and run the Move-Service with joint based procedure
        print("We will return to starting pose! Note, that the joints can have different values compared to the beginning, because we use the PosQuat Procedure which uses backwards-kinematic!")
        moving_robot_via_posquat(services, robot_values.pos2, robot_values.rot2)

        input("Finished! Press Enter to exit...")

    finally:
        # Disconnect from the server
        client.disconnect()

if __name__ == "__main__":
    main()