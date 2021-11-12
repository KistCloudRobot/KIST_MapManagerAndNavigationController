import sys
import time
import os
import pathlib
from threading import Condition, Thread
sys.path.append("D:\CloudRobot\Python-mcArbiFramework")
from arbi_agent.agent.arbi_agent import ArbiAgent
from arbi_agent.ltm.data_source import DataSource
from arbi_agent.agent import arbi_agent_executor
from arbi_agent.model import generalized_list_factory as GLFactory

sys.path.append(str(pathlib.Path(__file__).parent.parent.resolve()))

from MapManagement.MapMOS import MapMOS
from MapManagement.MapCloudlet import MapCloudlet
from DataType.RobotInfo import RobotInfo
from DataType.CallInfo import CallInfo

broker_url = "tcp://172.16.165.171:61313"
# broker_url = 'tcp://' + os.environ["JMS_BROKER"]


class MapManagerDataSource(DataSource):
    def __init__(self, broker_url):

        self.broker = broker_url # broker address
        self.connect(self.broker, "ds://www.arbi.com/Local/MapManager", 2) # connect broker

        self.map_file = str(pathlib.Path(__file__).parent.parent.resolve()) + "/data/map_cloud.txt" # load map file (~/data/map_cloud.txt)
        self.MAP = MapMOS(self.map_file) # interprete map

        self.sub_RobotInfo_ID = self.subscribe(
            "(rule (fact (RobotInfo $robot_id $x $y $loading $timestamp)) --> (notify (RobotInfo $robot_id $x $y $loading $timestamp)))")
            # RobotInfo subscription rule
        time.sleep(0.05)
        self.sub_DoorStatus_ID = self.subscribe("(rule (fact (DoorStatus $status)) --> (notify (DoorStatus $status)))")
            # DoorStatus subscription rule
        time.sleep(0.05)
        self.sub_MosPersonCall_ID = self.subscribe(
            "(rule (fact (MosPersonCall $locationID $callID)) --> (notify (MosPersonCall $locationID $callID)))")
            # PersonCall subscription rule
        time.sleep(0.05)

        self.AMR_IDs = ["AMR_LIFT1", "AMR_LIFT2", "AMR_TOW1", "AMR_TOW2"]  # AMR IDs
        self.AMR_LIFT_IDs = list()  # LIFT IDs
        self.AMR_TOW_IDs = list()  # TOW IDs
        for identifier in self.AMR_IDs:
            if "LIFT" in identifier:
                self.AMR_LIFT_IDs.append(identifier)
            elif "TOW" in identifier:
                self.AMR_TOW_IDs.append(identifier)

        self.AMR_LIFT_init = {"AMR_LIFT1": 201, "AMR_LIFT2": 202} # Initial vertex of LIFT
        self.AMR_TOW_init = {"AMR_TOW1": 203, "AMR_TOW2": 204} # Initial vertex of TOW

        self.Rack_LIFT_init = {'RACK_LIFT0': 5, 'RACK_LIFT1': 12, 'RACK_LIFT2': 14,
                               'RACK_LIFT3': 22, 'RACK_LIFT4': 18, 'RACK_LIFT5': 19}
        self.Rack_TOW_init = {'RACK_TOW0': 23, 'RACK_TOW1': 20}

        self.Cargo_init = {"CARGO0":  5, "CARGO1": 18, 'CARGO2': 19}
        self.Rack_TOW_init = {'RACK_TOW0': 21, 'RACK_TOW1': 20} # Initial vertex of TOW Rack

        self.Cargo_init = {"CARGO0":  5, "CARGO1": 18} # Initial vertex of Cargo
        self.Door_init = {'Door0': 0} # Initial status of Door
        self.MC = MapCloudlet(self.map_file, self.AMR_LIFT_init, self.AMR_TOW_init, self.Rack_TOW_init,
                              self.Rack_LIFT_init, self.Cargo_init, self.Door_init) # launch MC(MapCloudlet)

    def on_notify(self, content): # executed when the agent gets notification from ltm
        print("[on Notify]" + content)
        time.sleep(0.05)
        gl_notify = GLFactory.new_gl_from_gl_string(content)

        if gl_notify.get_name() == "RobotInfo": # "RobotInfo" notification from ltm
            ''' RobotInfo gl format: (RobotInfo $robot_id $x $y $loading $speed $battery)'''
            temp_RobotInfo = RobotInfo() # RobotInfo class
            temp_RobotInfo.id = gl_notify.get_expression(0).as_value().string_value() # get robotID
            temp_RobotInfo.pos = [gl_notify.get_expression(1).as_value().float_value(),
                                  gl_notify.get_expression(2).as_value().float_value()] # get current pose
            load = gl_notify.get_expression(3).as_value().string_value() # get current load state
            if load == "Unloading":
                load = 0 # Unloading -> 0
            elif load == "Loading":
                load = 1 # Loading -> 1
            else:
                load = -1 # Error
            temp_RobotInfo.load = load  # 1 for load , o for unload
            temp_RobotInfo.gl = gl_notify # save gl into RobotInfo class
            temp_RobotInfo.timestamp = gl_notify.get_expression(4).as_value().int_value() # get timestamp
            self.MC.update_MOS_robot_info(temp_RobotInfo) # update information of robot in MC

        elif gl_notify.get_name() == "DoorStatus": # "DoorStatus" notification from ltm
            ''' DoorStatus gl format: (DoorStatus $status)'''
            
            temp_DoorStatus = {}
            temp_DoorStatus['timestamp'] = int(time.time() * 1000) # timestamp dummy (to be handled in future)
            temp_DoorStatus['status'] = gl_notify.get_expression(0).as_value() # get door status
            self.MC.update_MOS_door_info(temp_DoorStatus) # update information of door in MC

        elif gl_notify.get_name() == "MosPersonCall":
            ''' MosPersonCall gl format: (HumanCall $LocationID, $CMD_ID)
                expression(0): LocationID
                expression(1): CMD_ID
                '''
            temp_CallInfo = CallInfo()
            temp_CallInfo.timestamp = int(time.time() * 1000) # timestamp dummy (to be handled in future)
            ### According to Protocol Document ###
            locationID = gl_notify.get_expression(0).as_value().int_value() # get locationID
            cmdID = gl_notify.get_expression(1).as_value().int_value() # get cmdID
            if locationID == 0:
                if cmdID == 0:
                    temp_CallInfo.vertex = [18, 18]
                    self.MC.call_LIFT(temp_CallInfo) # update information of person call in MC
                elif cmdID == 1:
                    temp_CallInfo.vertex = [19, 19]
                    self.MC.call_LIFT(temp_CallInfo) # update information of person call in MC
            elif locationID == 1 & cmdID == 3:
                self.MC.call_TOW(temp_CallInfo)
            elif locationID == 2:
                if cmdID == 4:
                    temp_CallInfo.vertex = [20, 20]
                    self.MC.call_removeCargo(temp_CallInfo) # update information of person call in MC
                elif cmdID == 5:
                    temp_CallInfo.vertex = [21, 21]
                    self.MC.call_removeCargo(temp_CallInfo) # update information of person call in MC


class MapManagerAgent(ArbiAgent):
    def __init__(self):
        super().__init__()
        self.lock = Condition()

        self.CM_name = "agent://www.arbi.com/Local/ContextManager"  # agent address of Local Context Manager (CM)
        self.TA_name = "agent://www.arbi.com/Local/TaskAllocator"  # agent address of Local Task Allocator (TA)
        self.NC_name = "agent://www.arbi.com/Local/NavigationController"  # agent address of Navigation Controller (NC)

        self.LIFT_CM_name = {
            "AMR_LIFT1": "agent://www.arbi.com/Lift1/ContextManager",
            "AMR_LIFT2": "agent://www.arbi.com/Lift2/ContextManager"
        } # agent address of LIFT Context Manager (CM)
        self.TOW_CM_name = {
            "AMR_TOW1": "agent://www.arbi.com/Tow1/ContextManager",
            "AMR_TOW2": "agent://www.arbi.com/Tow2/ContextManager"
        } # agent address of TOW Context Manager (CM)
        self.robot_goal = {
            "AMR_LIFT1": -1,
            "AMR_LIFT2": -1,
            "AMR_TOW1": -1,
            "AMR_TOW2": -1
        } # initialize robot goal
    def on_start(self): # executed when the agent initializes
        self.ltm = MapManagerDataSource(broker_url) # ltm class

        time.sleep(3)  # Wait for ltm initialization

        Thread(target=self.CargoPose_notify, args=(self.CM_name, ), daemon=True).start() # CargoPose Notify thread
        time.sleep(0.1)
        Thread(target=self.RackPose_notify, args=(self.CM_name, ), daemon=True).start() # RackPose Notify thread
        time.sleep(0.1)
        Thread(target=self.MultiRobotPose_notify, args=(self.NC_name, ), daemon=True).start() # MultiRobotPose Notify thread
        time.sleep(0.1)
        Thread(target=self.Collidable_notify, args=(self.NC_name, ), daemon=True).start() # Collidable Notify thread
        time.sleep(0.1)
        Thread(target=self.RobotPose_notify, args=(), daemon=True).start() # RobotPose Notify thread
        time.sleep(0.1)

    def close(self):
        self.ltm.unsubscribe(self.sub_RobotInfo_ID)
        self.ltm.unsubscribe(self.sub_DoorStatus_ID)
        self.ltm.unsubscribe(self.sub_MosPersonCall_ID)
        self.ltm.close()
        super().close()

    def on_notify(self, sender, notification): # executed when the agent gets notification
        print("[on Notify]", notification)
        time.sleep(0.05)
        temp_gl = GLFactory.new_gl_from_gl_string(notification)
        if temp_gl.get_name() == "RobotPathPlan":  # Notify from NC
            ''' RobotPathPlan gl format: (RobotPathPlan $robot_id $goal (path $v_id1 $v_id2 ….)) '''

            temp_path = []
            temp_gl_amr_id = temp_gl.get_expression(0).as_value().string_value() # get robotID
            temp_gl_goal = temp_gl.get_expression(1).as_value().string_value() # get goal
            self.robot_goal[temp_gl_amr_id] = temp_gl_goal # save goal
            temp_gl_path = temp_gl.get_expression(2).as_generalized_list() # get path

            temp_gl_path_size = temp_gl_path.get_expression_size()

            for i in range(temp_gl_path_size):
                temp_path.append(temp_gl_path.get_expression(i).as_value().int_value())
            # path gl to path list
            
            self.ltm.MC.insert_NAV_PLAN(temp_gl_amr_id, temp_path) # update information in MC
            time.sleep(0.05)
            self.ltm.MC.update_NAV_PLAN(temp_gl_amr_id) # update information in MC

    def CargoPose_notify(self, consumer): # Notify CargoPose every 1 sec to Local CM
        ''' CargoPose gl format: (CargoPose $cargo_id (vertex_id $v_id1 $v_id2) (on $robot_id $rack_id)) '''

        while True:
            time.sleep(1)
            temp_Cargo_info = self.ltm.MC.CARGO # get information of cargo in MC
            for id in temp_Cargo_info.keys():
                time.sleep(0.05)
                if id != -1: # check cargo is moving
                    ### Cargo_id != -1 ###
                    temp_Cargo_id = id
                    temp_Cargo_vertex = temp_Cargo_info[id]['vertex']
                    temp_Cargo_robot_id = temp_Cargo_info[id]['load_id'][0][0]
                    temp_Cargo_rack_id = temp_Cargo_info[id]['load_id'][0][1]
                    temp_Cargo_status = []

                    gl_template = "(CargoPose \"{cargo_id}\" (vertex_id {v_id1} {v_id2}) (on \"{robot_id}\" \"{rack_id}\") \"{status}\")"
                    gl = gl_template.format(
                        cargo_id=temp_Cargo_id,
                        v_id1=temp_Cargo_vertex[0][0],
                        v_id2=temp_Cargo_vertex[0][1],
                        robot_id=temp_Cargo_robot_id,
                        rack_id=temp_Cargo_rack_id,
                        status=temp_Cargo_status
                    )
                    self.notify(consumer, gl)
                    time.sleep(0.05)
                    print("cargo pose notify : " + gl)
                    self.ltm.assert_fact(gl)
                else:
                    pass

    def RackPose_notify(self, consumer): # Notify RackPose every 1 sec to Local CM
        ''' RackPose gl format: (RackPose $rack_id (vertex_id $v_id1 $v_id2) (on $robot_id $cargo_id)) '''

        while True:
            time.sleep(1)
            temp_RackPose_LIFT_info = self.ltm.MC.RACK_LIFT # get information of LIFT Rack in MC

            for id in temp_RackPose_LIFT_info.keys():
                time.sleep(0.05)
                if id != -1: # check rack is moving
                    ### Rack_id != -1 ###
                    temp_RackPose_LIFT_id = id
                    temp_RackPose_LIFT_vertex = temp_RackPose_LIFT_info[id]['vertex'][0]
                    temp_RackPose_LIFT_robot_id = temp_RackPose_LIFT_info[id]['load_id'][0][0]
                    temp_RackPose_LIFT_cargo_id = temp_RackPose_LIFT_info[id]['load_id'][0][1]
                    temp_RackPose_LIFT_status = []

                    gl_template = "(RackPose \"{rack_id}\" (vertex_id {v_id1} {v_id2}) (on \"{robot_id}\" \"{cargo_id}\") \"{status}\")"
                    gl = gl_template.format(
                        rack_id=temp_RackPose_LIFT_id,
                        v_id1=temp_RackPose_LIFT_vertex[0],
                        v_id2=temp_RackPose_LIFT_vertex[1],
                        robot_id=temp_RackPose_LIFT_robot_id,
                        cargo_id=temp_RackPose_LIFT_cargo_id,
                        status=temp_RackPose_LIFT_status
                    )
                    print("rack pose notify : " + gl)
                    self.notify(consumer, gl)
                else:
                    pass

            temp_RackPose_TOW_info = self.ltm.MC.RACK_TOW # get information of TOW Rack in MC
            for id in temp_RackPose_TOW_info.keys():
                time.sleep(0.05)
                if id != -1: # check rack is moving
                    ### Rack_id != -1 ###
                    temp_RackPose_TOW_id = id
                    temp_RackPose_TOW_vertex = temp_RackPose_TOW_info[id]['vertex'][0]
                    temp_RackPose_TOW_robot_id = temp_RackPose_TOW_info[id]['load_id'][0][0]
                    temp_RackPose_TOW_cargo_id = temp_RackPose_TOW_info[id]['load_id'][0][1]
                    temp_RackPose_TOW_status = []

                    gl_template = "(RackPose \"{rack_id}\" (vertex_id {v_id1} {v_id2}) (on \"{robot_id}\" \"{cargo_id}\") \"{status}\")"
                    gl = gl_template.format(
                        rack_id=temp_RackPose_TOW_id,
                        v_id1=temp_RackPose_TOW_vertex[0],
                        v_id2=temp_RackPose_TOW_vertex[1],
                        robot_id=temp_RackPose_TOW_robot_id,
                        cargo_id=temp_RackPose_TOW_cargo_id,
                        status=temp_RackPose_TOW_status
                    )
                    self.notify(consumer, gl)

                else:
                    pass

    def MultiRobotPose_notify(self, consumer): # Notify MultiRobotPose every 1 sec to NC
        ''' MultiRobotPose gl format: (MultiRobotPose (RobotPose $robot_id (vertex $vertex_id $vertex_id)), ...) '''

        while True:
            time.sleep(1)
            MultiRobotoPose_gl = "(MultiRobotPose"
            RobotPose_gl = " (RobotPose \"{robot_id}\" (vertex_id {v_id1} {v_id2}))"
            vertex_gl = "(vertex_id {v_id1} {v_id2})"
            for id in self.ltm.AMR_IDs:
                if "LIFT" in id:
                    temp_vertex = self.ltm.MC.AMR_LIFT[id]["vertex"][0]
                elif "TOW" in id:
                    temp_vertex = self.ltm.MC.AMR_TOW[id]["vertex"][0]

                MultiRobotoPose_gl += RobotPose_gl.format(
                    robot_id=id,
                    v_id1=temp_vertex[0],
                    v_id2=temp_vertex[1]
                )
            MultiRobotoPose_gl += ")"
            self.notify(consumer, MultiRobotoPose_gl)

    def RobotPose_notify(self): # Notify RobotPose every 1 sec to Robot CM
        ''' RobotPose gl format: (RobotAt $robot_id $v_id1 $v_id2)'''

        while True:
            time.sleep(1)
            temp_AMR_LIFT_info = self.ltm.MC.AMR_LIFT
            temp_AMR_TOW_info = self.ltm.MC.AMR_TOW

            for robot_id in temp_AMR_LIFT_info.keys():
                time.sleep(0.05)
                temp_AMR_LIFT_vertex = temp_AMR_LIFT_info[robot_id]["vertex"][0]
                gl_template = "(RobotAt \"{robot_id}\" {v_id1} {v_id2})"
                gl = gl_template.format(
                    robot_id=robot_id,
                    v_id1=temp_AMR_LIFT_vertex[0],
                    v_id2=temp_AMR_LIFT_vertex[1]
                )
                self.notify(self.LIFT_CM_name.get(robot_id), gl)

            for robot_id in temp_AMR_TOW_info.keys():
                time.sleep(0.05)
                temp_AMR_TOW_vertex = temp_AMR_TOW_info[robot_id]["vertex"][0]

                gl_template = "(RobotAt \"{robot_id}\" {v_id1} {v_id2})"
                gl = gl_template.format(
                    robot_id=robot_id,
                    v_id1=temp_AMR_TOW_vertex[0],
                    v_id2=temp_AMR_TOW_vertex[1]
                )
                self.notify(self.TOW_CM_name.get(robot_id), gl)

    def Collidable_notify(self, consumer): # Check Collidable every 2 sec and Notify if it has to NC
        ''' Collidable gl format: (Collidable $num (pair $robot_id $robot_id $time), …) '''

        while True:
            time.sleep(2)
            temp_Collidable_info = self.ltm.MC.detect_collision(10)
            temp_Collidable_num = len(temp_Collidable_info)
            if temp_Collidable_num:
                temp_notify = "(Collidable {num}"
                temp_Collidable_block = " (pair \"{robot_id1}\" \"{robot_id2}\" {time})"

                for i in range(temp_Collidable_num):
                    temp_robot_id_1 = temp_Collidable_info[i][0]
                    temp_robot_id_2 = temp_Collidable_info[i][1]

                    temp_notify = temp_notify + temp_Collidable_block.format(
                        robot_id1=temp_robot_id_1,
                        robot_id2=temp_robot_id_2,
                        time=temp_Collidable_info[i][2]
                    )
                temp_notify = temp_notify + ")"
                print(temp_notify.format(num=temp_Collidable_num))
                self.notify(consumer, temp_notify.format(num=temp_Collidable_num))

    # def DoorStatus_notify(self, consumer):
    #     while True:
    #         time.sleep(1)

    #         temp_Door_info = self.ltm.MC.Door
    #         temp_Door_status = temp_Door_info['Door0']['status']
    #         temp_gl = "(DoorStatus \"{status}\")"

    #         self.notify(consumer, temp_gl.format(status=temp_Door_status))

    # def RobotPathLeft_notify(self, consumer):
    #     temp_AMR_LIFT_Path = self.ltm.MC.Path_AMR_LIFT
    #     temp_AMR_TOW_Path = self.ltm.MC.Path_AMR_TOW

    #     for id in temp_AMR_LIFT_Path.keys():
    #         temp_notify = "(RobotPathLeft {robotID} {path})"
    #         temp_robot_id = self.ltm.AMR_IDs.index(id)
    #         temp_path = "(path"
    #         for i in range(len(temp_AMR_LIFT_Path[id])):
    #             temp_path = temp_path + str(temp_AMR_LIFT_Path[id][i])

    #         temp_path = temp_path + ")"

    #         self.notify(consumer, temp_notify.format(temp_robot_id, temp_path))

    #     for id in temp_AMR_TOW_Path.keys():
    #         temp_notify = "(RobotPathLeft {robotID} {path})"
    #         temp_robot_id = self.ltm.AMR_IDs.index(id)
    #         temp_path = "(path"
    #         for i in range(len(temp_AMR_TOW_Path[id])):
    #             temp_path = temp_path + str(temp_AMR_TOW_Path[id][i])

    #         temp_path = temp_path + ")"

    #         self.notify(consumer, temp_notify.format(temp_robot_id, temp_path))


    def on_query(self, sender, query): # executed when gets query
        print("on query : " + query)
        time.sleep(0.05)
        gl_query = GLFactory.new_gl_from_gl_string(query)
        query_name = gl_query.get_name()

        if query_name == "Collidable":  # From Local CM
            ''' Collidable gl format: (Collidable $num (pair $robot_id $robot_id $time), …) '''
            
            temp_Collidable_info = self.ltm.MC.detect_collision()
            temp_Collidable_num = len(temp_Collidable_info)
            temp_Collidable_block = " (pair \"{robot_id1}\" \"{robot_id2}\" {time})"

            temp_response = "(Collidable {num}"

            for i in range(temp_Collidable_num):
                temp_robot_id_1 = self.ltm.AMR_IDs.index(temp_Collidable_info[i][0])
                temp_robot_id_2 = self.ltm.AMR_IDs.index(temp_Collidable_info[i][1])

                temp_response = temp_response + temp_Collidable_block.format(robot_id1=temp_robot_id_1,
                                                                             robot_id2=temp_robot_id_2,
                                                                             time=temp_Collidable_info[i][2])

            temp_response = temp_response + ")"
            
            return temp_response.format(temp_Collidable_num)

        elif query_name == "RobotSpecInfo":  # From Local TA
            ''' RobotSpecInfo gl format: (RobotSpecInfo (RobotInfo $robot_id (vertex_id $v_id1 $v_id2) $load $goal), …) '''
            
            temp_robot_num = gl_query.get_expression_size()
            temp_response = "(RobotSpecInfo"
            temp_robotinfo_block = " (RobotInfo \"{robot_id}\" (vertex_id {v_id1} {v_id2}) {load} \"{goal}\")"

            for i in range(temp_robot_num):
                temp_robot_id = gl_query.get_expression(i).as_value().string_value()
                if "TOW" in temp_robot_id:
                    temp_TOW_info = self.ltm.MC.AMR_TOW[temp_robot_id]
                    temp_TOW_vertex = temp_TOW_info['vertex'][0]
                    temp_TOW_load = temp_TOW_info['load'][0]
                    temp_TOW_path = self.ltm.MC.Path_AMR_TOW[temp_robot_id]
                    if temp_TOW_path:
                        # temp_TOW_goal = temp_TOW_path[-1]
                        temp_TOW_goal = self.robot_goal[temp_robot_id]
                    elif not temp_TOW_path:
                        temp_TOW_goal = 0
                    temp_response += " (RobotInfo \"" + str(temp_robot_id) + "\"" + \
                                     " (vertex_id " + str(temp_TOW_vertex[0]) + " " + str(temp_TOW_vertex[1]) + ")" + \
                                     " " + str(temp_TOW_load) + " \"" + str(temp_TOW_goal) + "\")"
                elif "LIFT" in temp_robot_id:
                    temp_LIFT_info = self.ltm.MC.AMR_LIFT[temp_robot_id]
                    temp_LIFT_vertex = temp_LIFT_info['vertex'][0]
                    temp_LIFT_load = temp_LIFT_info['load'][0]
                    temp_LIFT_path = self.ltm.MC.Path_AMR_LIFT[temp_robot_id]
                    if temp_LIFT_path:
                        # temp_LIFT_goal = temp_LIFT_path[-1]
                        temp_TOW_goal = self.robot_goal[temp_robot_id]
                    elif not temp_LIFT_path:
                        temp_LIFT_goal = 0
                    else:
                        temp_LIFT_goal = 0
                    temp_response += " (RobotInfo \"" + str(temp_robot_id) + "\"" + \
                                     " (vertex_id " + str(temp_LIFT_vertex[0]) + " " + str(temp_LIFT_vertex[1]) + ")" + \
                                     " " + str(temp_LIFT_load) + " \"" + str(temp_LIFT_goal) + "\")"
            temp_response += ")"
            return temp_response

        else:
            return "(fail)"


if __name__ == "__main__":
    agent = MapManagerAgent()
    arbi_agent_executor.execute(broker_url=broker_url, agent_name="agent://www.arbi.com/Local/MapManager",
                               agent=agent, broker_type=2)  # same role with agent.initialize

    while True:
        time.sleep(1)

    agent.close()

