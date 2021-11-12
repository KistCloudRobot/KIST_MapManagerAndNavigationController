import sys
import time
import os
import pathlib
from threading import Condition, Thread
sys.path.append("D:\CloudRobot\Python-mcArbiFramework")
from arbi_agent.agent.arbi_agent import ArbiAgent
from arbi_agent.ltm.data_source import DataSource
from arbi_agent.agent import arbi_agent_excutor
from arbi_agent.model import generalized_list_factory as GLFactory

sys.path.append(str(pathlib.Path(__file__).parent.parent.resolve()))

from MapManagement.MapMOS import MapMOS
from MapManagement.MapCloudlet import MapCloudlet
from DataType.RobotInfo import RobotInfo
from DataType.CallInfo import CallInfo

broker_url = "tcp://127.0.0.1:61316"
# broker_url = 'tcp://' + os.environ["JMS_BROKER"]


class MapManagerDataSource(DataSource):
    def __init__(self, broker_url):

        self.broker = broker_url
        self.connect(self.broker, "ds://www.arbi.com/Local/MapManager", 2)

        self.map_file = str(pathlib.Path(__file__).parent.parent.resolve()) + "/data/map_cloud.txt"
        self.MAP = MapMOS(self.map_file)

        self.sub_RobotInfo_ID = self.subscribe(
            "(rule (fact (RobotInfo $robot_id $x $y $loading $timestamp)) --> (notify (RobotInfo $robot_id $x $y $loading $timestamp)))")
        time.sleep(0.05)
        self.sub_DoorStatus_ID = self.subscribe("(rule (fact (DoorStatus $status)) --> (notify (DoorStatus $status)))")
        time.sleep(0.05)
        self.sub_MosPersonCall_ID = self.subscribe(
            "(rule (fact (MosPersonCall $locationID $callID)) --> (notify (MosPersonCall $locationID $callID)))")
        time.sleep(0.05)

        self.AMR_LIFT_IDs = ["AMR_LIFT1", "AMR_LIFT2"]
        self.AMR_TOW_IDs = ["AMR_TOW1", "AMR_TOW2"]

        self.AMR_IDs = ["AMR_LIFT1", "AMR_LIFT2", "AMR_TOW1", "AMR_TOW2"]

        self.AMR_LIFT_init = {"AMR_LIFT1": 201, "AMR_LIFT2": 202}
        self.AMR_TOW_init = {"AMR_TOW1": 203, "AMR_TOW2": 204}

        self.Rack_LIFT_init = {'RACK_LIFT0': 5, 'RACK_LIFT1': 13, 'RACK_LIFT2': 14,
                               'RACK_LIFT3': 15, 'RACK_LIFT4': 18, 'RACK_LIFT5': 19}
        self.Rack_TOW_init = {'RACK_TOW0': 21, 'RACK_TOW1': 20}

        self.Cargo_init = {"CARGO0":  5, "CARGO1": 18}

        self.Door_init = {'Door0': 0}
        self.MM = MapCloudlet(self.map_file, self.AMR_LIFT_init, self.AMR_TOW_init, self.Rack_TOW_init,
                              self.Rack_LIFT_init, self.Cargo_init, self.Door_init)

    def on_notify(self, content):
        print("on notify! " + content)
        time.sleep(0.05)
        gl_notify = GLFactory.new_gl_from_gl_string(content)

        if gl_notify.get_name() == "RobotInfo":
            temp_RobotInfo = RobotInfo()
            temp_RobotInfo.id = gl_notify.get_expression(0).as_value().string_value()
            temp_RobotInfo.pos = [gl_notify.get_expression(1).as_value().float_value(),
                                  gl_notify.get_expression(2).as_value().float_value()]
            # print(temp_RobotInfo.id, temp_RobotInfo.pos)
            load = gl_notify.get_expression(3).as_value().string_value()
            if load == "Unloading":
                load = 0
            elif load == "Loading":
                load = 1
            else:
                load = -1
            temp_RobotInfo.load = load  # 1 for load , o for unload
            temp_RobotInfo.gl = gl_notify
            temp_RobotInfo.timestamp = gl_notify.get_expression(4).as_value().int_value()
            self.MM.update_MOS_robot_info(temp_RobotInfo)

        elif gl_notify.get_name() == "DoorStatus":
            temp_DoorStatus = {}
            # timestamp dummy
            temp_DoorStatus['timestamp'] = int(time.time() * 1000)
            temp_DoorStatus['status'] = gl_notify.get_expression(0).as_value()  ### Status Type Check ###
            self.MM.update_MOS_door_info(temp_DoorStatus)

        elif gl_notify.get_name() == "MosPersonCall":
            temp_CallInfo = CallInfo()
            # timestamp dummy
            temp_CallInfo.timestamp = int(time.time() * 1000)
            ### expression(0): LocationID ###
            ### expression(1): CMD_ID ###
            ### According to Protocol Document ###
            locationID = gl_notify.get_expression(0).as_value().int_value()
            cmdID = gl_notify.get_expression(1).as_value().int_value()
            if locationID == 0:
                if cmdID == 0:
                    temp_CallInfo.vertex = [18, 18]
                    self.MM.call_LIFT(temp_CallInfo)
                elif cmdID == 1:
                    temp_CallInfo.vertex = [19, 19]
                    self.MM.call_LIFT(temp_CallInfo)

            elif locationID == 1 & cmdID == 3:
                self.MM.call_TOW(temp_CallInfo)

            elif locationID == 2:
                if cmdID == 4:
                    temp_CallInfo.vertex = [20, 20]
                    self.MM.call_removeCargo(temp_CallInfo)
                elif cmdID == 5:
                    temp_CallInfo.vertex = [21, 21]
                    self.MM.call_removeCargo(temp_CallInfo)


class MapManagerAgent(ArbiAgent):
    def __init__(self):
        super().__init__()
        self.lock = Condition()

        self.CM_name = "agent://www.arbi.com/Local/ContextManager"  # name of Local-ContextManager Agent
        self.TA_name = "agent://www.arbi.com/Local/TaskAllocator"  # name of Local-TaskAllocation Agent
        self.NC_name = "agent://www.arbi.com/Local/NavigationController"  # name of Overall-NavigationConrtoller Agent

        self.LIFT_CM_name = {
            "AMR_LIFT1": "agent://www.arbi.com/Lift1/ContextManager",
            "AMR_LIFT2": "agent://www.arbi.com/Lift2/ContextManager"
        }
        self.TOW_CM_name = {
            "AMR_TOW1": "agent://www.arbi.com/Tow1/ContextManager",
            "AMR_TOW2": "agent://www.arbi.com/Tow2/ContextManager"
        }
        self.robot_goal = {
            "AMR_LIFT1": -1,
            "AMR_LIFT2": -1,
            "AMR_TOW1": -1,
            "AMR_TOW2": -1
        }
    def on_start(self):
        self.ltm = MapManagerDataSource(broker_url)

        time.sleep(3)  # Wait for ltm initialization

        Thread(target=self.CargoPose_notify, args=(self.CM_name, ), daemon=True).start()
        time.sleep(0.1)
        Thread(target=self.RackPose_notify, args=(self.CM_name, ), daemon=True).start()
        time.sleep(0.1)
        Thread(target=self.MultiRobotPose_notify, args=(self.NC_name, ), daemon=True).start()
        time.sleep(0.1)
        Thread(target=self.Collidable_notify, args=(self.NC_name, ), daemon=True).start()
        time.sleep(0.1)
        Thread(target=self.RobotPose_notify, args=(), daemon=True).start()
        time.sleep(0.1)

    def close(self):
        self.ltm.unsubscribe(self.sub_RobotInfo_ID)
        self.ltm.unsubscribe(self.sub_DoorStatus_ID)
        self.ltm.unsubscribe(self.sub_MosPersonCall_ID)
        self.ltm.close()
        super().close()

    def on_notify(self, sender, notification):
        print("on notify! ", notification)
        time.sleep(0.05)
        temp_gl = GLFactory.new_gl_from_gl_string(notification)
        if temp_gl.get_name() == "RobotPathPlan":  # Notify from Navigation Controller Agent
            temp_path = []
            temp_gl_amr_id = temp_gl.get_expression(0).as_value().string_value()
            temp_gl_goal = temp_gl.get_expression(1).as_value().string_value()
            self.robot_goal[temp_gl_amr_id] = temp_gl_goal
            temp_gl_path = temp_gl.get_expression(2).as_generalized_list()

            temp_gl_path_size = temp_gl_path.get_expression_size()

            for i in range(temp_gl_path_size):
                temp_path.append(temp_gl_path.get_expression(i).as_value().int_value())

            self.ltm.MM.insert_NAV_PLAN(temp_gl_amr_id, temp_path)
            time.sleep(0.05)
            self.ltm.MM.update_NAV_PLAN(temp_gl_amr_id)

    def CargoPose_notify(self, consumer):
        while True:
            time.sleep(1)
            temp_Cargo_info = self.ltm.MM.CARGO
            for id in temp_Cargo_info.keys():
                time.sleep(0.05)
                if id != -1:
                    ### Cargo_id != -1 ###
                    temp_Cargo_id = id
                    temp_Cargo_vertex = temp_Cargo_info[id]['vertex']
                    temp_Cargo_robot_id = temp_Cargo_info[id]['load_id'][0][0]
                    temp_Cargo_rack_id = temp_Cargo_info[id]['load_id'][0][1]
                    temp_Cargo_status = []

                    # TODO LIFT STATUS
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

    def RackPose_notify(self, consumer):
        while True:
            time.sleep(1)
            temp_RackPose_LIFT_info = self.ltm.MM.RACK_LIFT

            for id in temp_RackPose_LIFT_info.keys():
                time.sleep(0.05)
                if id != -1:
                    ### Rack_id != -1 ###
                    temp_RackPose_LIFT_id = id
                    temp_RackPose_LIFT_vertex = temp_RackPose_LIFT_info[id]['vertex'][0]
                    temp_RackPose_LIFT_robot_id = temp_RackPose_LIFT_info[id]['load_id'][0][0]
                    temp_RackPose_LIFT_cargo_id = temp_RackPose_LIFT_info[id]['load_id'][0][1]
                    temp_RackPose_LIFT_status = []

                    # TODO LIFT STATUS 처리
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

            temp_RackPose_TOW_info = self.ltm.MM.RACK_TOW
            for id in temp_RackPose_TOW_info.keys():
                time.sleep(0.05)
                if id != -1:
                    ### Rack_id != -1 ###
                    temp_RackPose_TOW_id = id
                    temp_RackPose_TOW_vertex = temp_RackPose_TOW_info[id]['vertex'][0]
                    temp_RackPose_TOW_robot_id = temp_RackPose_TOW_info[id]['load_id'][0][0]
                    temp_RackPose_TOW_cargo_id = temp_RackPose_TOW_info[id]['load_id'][0][1]
                    temp_RackPose_TOW_status = []

                    # TODO LIFT STATUS 처리
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

    def MultiRobotPose_notify(self, consumer):
        while True:
            time.sleep(1)
            MultiRobotoPose_gl = "(MultiRobotPose"
            RobotPose_gl = " (RobotPose \"{robot_id}\" (vertex_id {v_id1} {v_id2}))"
            vertex_gl = "(vertex_id {v_id1} {v_id2})"
            for id in self.ltm.AMR_IDs:
                if "LIFT" in id:
                    temp_vertex = self.ltm.MM.AMR_LIFT[id]["vertex"][0]
                elif "TOW" in id:
                    temp_vertex = self.ltm.MM.AMR_TOW[id]["vertex"][0]

                MultiRobotoPose_gl += RobotPose_gl.format(
                    robot_id=id,
                    v_id1=temp_vertex[0],
                    v_id2=temp_vertex[1]
                )
            MultiRobotoPose_gl += ")"
            self.notify(consumer, MultiRobotoPose_gl)

    def RobotPose_notify(self):
        while True:
            time.sleep(1)
            temp_AMR_LIFT_info = self.ltm.MM.AMR_LIFT
            temp_AMR_TOW_info = self.ltm.MM.AMR_TOW

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

    def Collidable_notify(self, consumer):
        while True:
            time.sleep(2)
            temp_Collidable_info = self.ltm.MM.detect_collision(10)
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

    #         temp_Door_info = self.ltm.MM.Door
    #         temp_Door_status = temp_Door_info['Door0']['status']
    #         temp_gl = "(DoorStatus \"{status}\")"

    #         self.notify(consumer, temp_gl.format(status=temp_Door_status))

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


    def on_query(self, sender, query):
        print("on query : " + query)
        time.sleep(0.05)
        gl_query = GLFactory.new_gl_from_gl_string(query)
        query_name = gl_query.get_name()

        if query_name == "Collidable":  # From Local CM
            temp_Collidable_info = self.ltm.MM.detect_collision()
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
            # self.send(sender, temp_response.format(temp_Collidable_num))
            return temp_response.format(temp_Collidable_num)

        elif query_name == "RobotSpecInfo":  # From Local TA
            temp_robot_num = gl_query.get_expression_size()
            temp_response = "(RobotSpecInfo"
            temp_robotinfo_block = " (RobotInfo \"{robot_id}\" (vertex_id {v_id1} {v_id2}) {load} \"{goal}\")"

            for i in range(temp_robot_num):
                temp_robot_id = gl_query.get_expression(i).as_value().string_value()
                if "TOW" in temp_robot_id:
                    temp_TOW_info = self.ltm.MM.AMR_TOW[temp_robot_id]
                    temp_TOW_vertex = temp_TOW_info['vertex'][0]
                    temp_TOW_load = temp_TOW_info['load'][0]
                    temp_TOW_path = self.ltm.MM.Path_AMR_TOW[temp_robot_id]
                    if temp_TOW_path:
                        # temp_TOW_goal = temp_TOW_path[-1]
                        temp_TOW_goal = self.robot_goal[temp_robot_id]
                    elif not temp_TOW_path:
                        temp_TOW_goal = 0
                    temp_response += " (RobotInfo \"" + str(temp_robot_id) + "\"" + \
                                     " (vertex_id " + str(temp_TOW_vertex[0]) + " " + str(temp_TOW_vertex[1]) + ")" + \
                                     " " + str(temp_TOW_load) + " \"" + str(temp_TOW_goal) + "\")"
                elif "LIFT" in temp_robot_id:
                    temp_LIFT_info = self.ltm.MM.AMR_LIFT[temp_robot_id]
                    temp_LIFT_vertex = temp_LIFT_info['vertex'][0]
                    temp_LIFT_load = temp_LIFT_info['load'][0]
                    temp_LIFT_path = self.ltm.MM.Path_AMR_LIFT[temp_robot_id]
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
    arbi_agent_excutor.execute(broker_url=broker_url, agent_name="agent://www.arbi.com/Local/MapManager",
                               agent=agent, broker_type=2)  # same role with agent.initialize

    while True:
        time.sleep(1)

    agent.close()

