import os
import sys
import time
sys.path.append("D:\git_ws\CloudRobot") ### TEMP ###
import threading
from threading import Condition
from arbi_agent.agent import arbi_agent

from arbi_agent.agent.arbi_agent import ArbiAgent
from arbi_agent.configuration import BrokerType
from arbi_agent.ltm.data_source import DataSource
from arbi_agent.agent import arbi_agent_excutor
from arbi_agent.model import generalized_list_factory as GLFactory

from MapManagement.MapMOS import MapMOS
from MapManagement.MapCloudlet import MapCloudlet
from DataType.RobotInfo import RobotInfo
from DataType.CallInfo import CallInfo

class MapManagerDataSource(DataSource):
    def __init__(self, broker_url):
        
        self.broker = broker_url
        self.connect(self.broker, "", BrokerType.ZERO_MQ)
        
        self.map_file = "../data/map_cloud.txt"
        self.MAP = MapMOS(self.map_file)
        
        self.sub_RobotInfo_ID = self.subscribe("(rule (fact (RobotInfo $robot_id $x $y $loading $speed $battery)) --> (notify (RobotInfo $robot_id $x $y $loading $speed $battery)))")
        self.sub_DoorStatus_ID = self.subscribe("(rule (fact (DoorStatus $status)) --> (notify (DoorStatus $status)))")

        self.AMR_LIFT_IDs = ["AMRLIFT0", "AMRLIFT1"]
        self.AMR_TOW_IDs = ["AMRTOW0", "AMRTOW1"]
        
        self.AMR_IDs = ["AMRLIFT0", "AMRLIFT1", "AMRTOW0", "AMRTOW1"]

        self.AMR_LIFT_init = {"AMRLIFT0": 0, "AMRLIFT1": 0}
        self.AMR_TOW_init = {"AMRTOW0": 0, "AMRTOW1": 0}
        
        self.Rack_LIFT_init = {'RACKLIFT0':0, 'RACKLIFT1':0, 'RACKLIFT2':0, 'RACKLIFT3':0}
        self.Rack_TOW_init = {'RACKTOW0':0, 'RACKTOW1':0}
        
        self.Door_init = {'Door0':0}

        self.MM = MapCloudlet(self.map_file, self.AMR_TOW_init, self.AMR_TOW_init, self.Rack_TOW_init, self.Rack_LIFT_init, self.Door_init)
        
    def on_notify(self, content):
        gl_notify = GLFactory.new_gl_from_gl_string(content)
        gl_timestamp = self.get_full_message().get_timestamp()
        ### Timestamp -> Not Assigned ###
        ### RobotInfo / HumanCall part needs Timestamp ###
        if gl_notify.get_name() == "RobotInfo":
            ### Format Not Assigned
            temp_RobotInfo = RobotInfo()
            temp_RobotInfo.id = self.AMR_IDs[gl_notify.get_expression(0).as_value()]
            temp_RobotInfo.pos = [gl_notify.get_expression(1).as_value(), gl_notify.get_expression(2).as_value()]
            temp_RobotInfo.load = gl_notify.get_expression(3).as_value() # 1 for load , o for unload
            temp_RobotInfo.gl = gl_notify
            ### Timestamp ###
            temp_RobotInfo.timestamp = gl_timestamp
            self.MM.update_MOS_robot_info(temp_RobotInfo)
            
        elif gl_notify.get_name() == "DoorStatus":
            temp_DoorStatus = {}
            temp_DoorStatus['timestamp'] = gl_timestamp
            temp_DoorStatus['status'] = gl_notify.get_expression(0).as_value() ### Status Type Check ###
            self.MM.update_MOS_door_info(temp_DoorStatus)
        
        elif gl_notify.get_name() == "HumanCall":
            temp_CallInfo = CallInfo()
            temp_CallInfo.timestamp = gl_timestamp
            ### expression(0): LocationID ###
            ### expression(1): CMD_ID ###
            ### According to Protocol Document ###
            if gl_notify.get_expression(0) == 0: 
                if gl_notify.get_expression(1) == 0:
                    temp_CallInfo.vertex = [18, 18]
                    self.MM.call_LIFT(temp_CallInfo)
                elif gl_notify.get_expression(1) == 1:
                    temp_CallInfo.vertex = [19, 19]
                    self.MM.call_LIFT(temp_CallInfo)

            elif gl_notify.get_expression(0) == 1 & gl_notify.get_expression(1) == 3:
                self.MM.call_TOW(temp_CallInfo)

            elif gl_notify.get_expression(0) == 2:
                if gl_notify.get_expression(1) == 4:
                    temp_CallInfo.vertex = [20, 20]
                    self.MM.call_removeCargo(temp_CallInfo)
                elif gl_notify.get_expression(1) == 5:
                    temp_CallInfo.vertex = [21, 21]
                    self.MM.call_removeCargo(temp_CallInfo)
            
class MapManagerAgent(ArbiAgent):
    def __init__(self):
        super().__init__()
        self.lock = Condition()

        self.CM_name = "agent://www.arbi.com/ContextManagerAgent" # name of Local-ContextManager Agent
        self.TA_name = "agent://www.arbi.com/TaskAllocationAgent" # name of Local-TaskAllocation Agent
        self.NC_name = "agent://www.arbi.com/NavigationControllerAgent" # name of Overall-NavigationConrtoller Agent
        
        self.threading_timer = 1

        self.robot_goal = {}

    def on_start(self):
        self.ltm = MapManagerDataSource()
        self.ltm.connect("tcp://127.0.0.1:61616", "", BrokerType.ZERO_MQ)

        time.sleep(10) # Wait for ltm initialization

        # Notification Thread
        self.CargoPose_notify(self.CM_name)
        self.RackPose_notify(self.CM_name)
        self.RobotPose_notify(self.TA_name)
        self.RobotPose_notify(self.NC_name)
        self.Collidable_notify(self.NC_name)
        self.DoorStatus_notify(self.CM_name)
        # self.RobotPathLeft_notify(self.NC_name)
    
    def on_notify(self, sender, notification):
        temp_gl = GLFactory.new_gl_from_gl_string(notification)
        
        if temp_gl.get_name() == "RobotPathPlan": # Notify from Navigation Controller Agent
            temp_path = []
            temp_gl_amr_id = temp_gl.get_expression(0).as_value()
            temp_gl_goal = temp_gl.get_expression(1).as_value()
            self.robot_goal[temp_gl_amr_id] = temp_gl_goal
            temp_gl_path = temp_gl.get_expression(2).as_generalized_list()

            temp_gl_path_size = temp_gl_path.get_expression_size()

            for i in range(temp_gl_path_size):
                temp_path.append(temp_gl_path.get_expression(i))
        
            self.ltm.MM.insert_NAV_PLAN(temp_gl_amr_id, temp_path)
    
    def CargoPose_notify(self, consumer):
        temp_Cargo_info = self.ltm.MM.CARGO

        for id in temp_Cargo_info.keys():
            if id != -1:
                ### Cargo_id != -1 ###
                temp_Cargo_id = id
                temp_Cargo_vertex = temp_Cargo_info[id]['vertex']
                temp_Cargo_robot_id = temp_Cargo_info[id]['load_id'][0]
                temp_Cargo_rack_id = temp_Cargo_info[id]['load_id'][1]
                temp_Cargo_status = []

                temp_notify = "(CargoPose {cargo_id} (vertex_id {v_id1} {v_id2}) (on {robot_id} {rack_id}) {status})"

                self.notify(consumer, temp_notify.format(temp_Cargo_id, temp_Cargo_vertex[0], temp_Cargo_vertex[1], temp_Cargo_robot_id, temp_Cargo_rack_id, temp_Cargo_status))
                self.ltm.assert_fact()
            else:
                pass

        threading.Timer(self.threading_timer, self.CargoPose_notify, [consumer]).start()
    
    def RackPose_notify(self, consumer):
        temp_RackPose_LIFT_info = self.ltm.MM.RACK_LIFT
        for id in temp_RackPose_LIFT_info.keys():
            if id != -1:
                ### Rack_id != -1 ###
                temp_RackPose_LIFT_id = id
                temp_RackPose_LIFT_vertex = temp_RackPose_LIFT_info[id]['vertex']
                temp_RackPose_LIFT_robot_id = temp_RackPose_LIFT_info[id]['load_id'][0]
                temp_RackPose_LIFT_cargo_id = temp_RackPose_LIFT_info[id]['load_id'][1]
                temp_RackPose_LIFT_status = []

                temp_notify = "(RackPose {rack_id} (vertex_id {v_id1} {v_id2}) (on {robot_id} {cargo_id}) {status})"
                
                self.notify(consumer, temp_notify.format(temp_RackPose_LIFT_id, temp_RackPose_LIFT_vertex[0], temp_RackPose_LIFT_vertex[1], \
                    temp_RackPose_LIFT_robot_id, temp_RackPose_LIFT_cargo_id, temp_RackPose_LIFT_status))
            else:
                pass

        temp_RackPose_TOW_info = self.ltm.MM.RACK_TOW
        for id in temp_RackPose_TOW_info.keys():
            if id != -1:
                ### Rack_id != -1 ###
                temp_RackPose_TOW_id = id
                temp_RackPose_TOW_vertex = temp_RackPose_TOW_info[id]['vertex']
                temp_RackPose_TOW_robot_id = temp_RackPose_TOW_info[id]['load_id'][0]
                temp_RackPose_TOW_cargo_id = temp_RackPose_TOW_info[id]['load_id'][1]
                temp_RackPose_TOW_status = []

                temp_notify = "(RackPose {rack_id} (vertex_id {v_id1} {v_id2}) (on {robot_id} {cargo_id}) {status})"
                
                self.notify(consumer, temp_notify.format(temp_RackPose_TOW_id, temp_RackPose_TOW_vertex[0], temp_RackPose_TOW_vertex[1], \
                    temp_RackPose_TOW_robot_id, temp_RackPose_TOW_cargo_id, temp_RackPose_TOW_status))
            else:
                pass
        threading.Timer(self.threading_timer, self.RackPose_notify, [consumer]).start()
    
    def RobotPose_notify(self, consumer):
        temp_AMR_LIFT_info = self.ltm.MM.AMR_LIFT
        temp_AMR_TOW_info = self.ltm.MM.AMR_TOW

        for id in temp_AMR_LIFT_info.keys():
            temp_AMR_LIFT_id = self.ltm.AMR_IDs.index(id)
            temp_AMR_LIFT_vertex = temp_AMR_LIFT_info[id]['vertex']

            temp_notify = "(RobotPose {robot_id} (vertex_id {v_id1} {v_id2}))"

            self.notify(consumer, temp_notify.format(temp_AMR_LIFT_id, temp_AMR_LIFT_vertex[0], temp_AMR_LIFT_vertex[1]))

        for id in temp_AMR_TOW_info.keys():
            temp_AMR_TOW_id = self.ltm.AMR_IDs.index(id)
            temp_AMR_TOW_vertex = temp_AMR_TOW_info[id]['vertex']

            temp_notify = "(RobotPose {robot_id} (vertex_id {v_id1} {v_id2}))"

            self.notify(consumer, temp_notify.format(temp_AMR_TOW_id, temp_AMR_TOW_vertex[0], temp_AMR_TOW_vertex[1]))

        threading.Timer(self.threading_timer, self.RobotPose_notify, [consumer]).start()
    
    def Collidable_notify(self, consumer):
        ### Timestamp ###
        temp_Collidable_info = self.ltm.MM.detect_collision()
        temp_Collidable_num = len(temp_Collidable_info)
        temp_Collidable_block = " (pair {robot_id} {robot_id} {time})"
        
        temp_notify = "(Collidable {num}"
        
        for i in range(temp_Collidable_num):
            temp_robot_id_1 = self.ltm.AMR_IDs.index(temp_Collidable_info[i][0])
            temp_robot_id_2 = self.ltm.AMR_IDs.index(temp_Collidable_info[i][1])

            temp_notify = temp_notify + temp_Collidable_block.format(temp_robot_id_1, temp_robot_id_2, temp_Collidable_info[i][2])

        temp_notify = temp_notify + ")"

        self.notify(consumer, temp_notify.format(temp_Collidable_num))

        threading.Timer(self.threading_timer, self.Collidable_notify, [consumer]).start()

    def DoorStatus_notify(self, consumer):

        temp_Door_info = self.ltm.MM.Door
        temp_Door_status = temp_Door_info[0]['status']
        temp_gl = "(DoorStatus {status})"

        self.notify(consumer, temp_gl.format(temp_Door_status))
        threading.Timer(self.threading_timer, self.DoorStatus_notify, [consumer]).start()

    # def RobotPathLeft_notify(self, consumer):
    #     temp_AMR_LIFT_Path = self.ltm.MM.Path_AMR_LIFT
    #     temp_AMR_TOW_Path = self.ltm.MM.Path_AMR_TOW

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
        
    #     threading.Timer(self.threading_timer, self.RobotPathLeft_notify, [consumer]).start()

    def on_query(self, sender, query):
        gl_query = GLFactory.new_gl_from_gl_string(query)

        if gl_query.get_name() == "Collidable": # From Local CM
            ### Timestamp ###
            temp_Collidable_info = self.ltm.MM.detect_collision()
            temp_Collidable_num = len(temp_Collidable_info)
            temp_Collidable_block = " (pair {robot_id} {robot_id} {time})"
        
            temp_response = "(Collidable {num}"
        
            for i in range(temp_Collidable_num):
                temp_robot_id_1 = self.ltm.AMR_IDs.index(temp_Collidable_info[i][0])
                temp_robot_id_2 = self.ltm.AMR_IDs.index(temp_Collidable_info[i][1])

                temp_response = temp_response + temp_Collidable_block.format(temp_robot_id_1, temp_robot_id_2, temp_Collidable_info[i][2])

            temp_response = temp_response + ")"
            # self.send(sender, temp_response.format(temp_Collidable_num))
            return temp_response.format(temp_Collidable_num)

        elif gl_query.get_name() == "RobotSpecInfo": # From Local TA
            temp_robot_num = gl_query.get_expression_size()
            temp_response = "(RobotSpecInfo"
            temp_robotinfo_block = " (RobotInfo {robot_id} (vertex_id {v_id1} {v_id2}) {load} {goal})"

            for i in range(temp_robot_num):
                temp_robot_id = gl_query.get_expression(i).as_value()
                if "TOW" in temp_robot_id:
                    temp_TOW_info = self.ltm.MM.AMR_TOW[temp_robot_id]
                    temp_TOW_vertex = temp_TOW_info['vertex']
                    temp_TOW_load = temp_TOW_info['load']
                    temp_TOW_path = self.ltm.MM.Path_AMR_TOW[temp_robot_id]
                    if temp_TOW_path:
                        temp_TOW_goal = temp_TOW_path[-1]
                    elif not temp_TOW_path:
                        temp_TOW_goal = 0
                    robot_id = "\"" + temp_robot_id + "\""
                    temp_response += temp_robotinfo_block.format(robot_id, temp_TOW_vertex[0], temp_TOW_vertex[1], temp_TOW_load, temp_TOW_goal)

                elif "LIST" in temp_robot_id:
                    temp_LIST_info = self.ltm.MM.AMR_LIST[temp_robot_id]
                    temp_LIST_vertex = temp_LIST_info['vertex']
                    temp_LIST_load = temp_LIST_info['load']
                    temp_LIST_path = self.ltm.MM.Path_AMR_LIST[temp_robot_id]
                    if temp_LIST_path:
                        temp_LIST_goal = temp_LIST_path[-1]
                    elif not temp_LIST_path:
                        temp_LIST_goal = 0
                    robot_id = "\"" + temp_robot_id + "\""
                    temp_response += temp_robotinfo_block.format(robot_id, temp_LIST_vertex[0], temp_LIST_vertex[1], temp_LIST_load, temp_LIST_goal)

            temp_response += ")"
            return temp_response

        
if __name__ == "__main__":
    agent = MapManagerAgent()
    arbi_agent_excutor.execute(broker_url="tcp://127.0.0.1:61616", agent_name="agent://www.arbi.com/MapManagerAgent", agent=agent, broker_type=2) # same role with agent.initialize
