from asyncua import Client

def robot_ctrl():
    opcua_client = Client(url='opc.tcp://127.0.0.1:4840/')
    opcua_client.connect()

    print(opcua_client.get_objects_node(ns=3, s='services.Hand-Service.op_src_mode.StateOpOp'))
    



if __name__ == '__main__':
    robot_ctrl()