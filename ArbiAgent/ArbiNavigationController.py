import os
import sys
import time
from threading import Condition, Thread
from arbi_agent.agent import arbi_agent

from arbi_agent.agent.arbi_agent import ArbiAgent
from arbi_agent.configuration import BrokerType
from arbi_agent.ltm.data_source import DataSource
from arbi_agent.agent import arbi_agent_excutor
from arbi_agent.model import generalized_list_factory as GLFactory

from MapManagement.MapMOS import MapMOS
from NavigationControl.NavigationControl import NavigationControl


class NavigationControlerDataSource(DataSource):
    def __init__(self, broker_url):
        
        self.broker = broker_url
        self.connect(self.broker, "ds://www.arbi.com/Local/NavigationController", 2)

        self.map_file = "../data/map_cloud.txt"
        self.MAP = MapMOS(self.map_file)
        
        self.AMR_IDs = ["AMR_LIFT1", "AMR_LIFT2", "AMR_TOW1", "AMR_TOW2"]
        self.AMR_LIFT_IDs = list()
        self.AMR_TOW_IDs = list()
        for identifier in self.AMR_IDs:
            if "LIFT" in identifier:
                self.AMR_LIFT_IDs.append(identifier)
            elif "TOW" in identifier:
                self.AMR_TOW_IDs.append(identifier)

        self.NC = NavigationControl(self.AMR_IDs, self.AMR_LIFT_IDs, self.AMR_TOW_IDs)


class NavigationControlerAgent(ArbiAgent):
    def __init__(self):
        super().__init__()
        self.lock = Condition()
        self.TA_name = "agent://TM" # name of MAPF
        # TODO LIFT, TOW TM name dict로 수정된거 할당
        self.LIFT_TM_name = {
            "AMR_LIFT1": "agent://www.arbi.com/Lift1/TaskManager",
            "AMR_LIFT2": "agent://www.arbi.com/Lift2/TaskManager"
        }
        self.TOW_TM_name = {
            "AMR_TOW1": "agent://www.arbi.com/Tow1/TaskManager",
            "AMR_TOW2": "agent://www.arbi.com/Tow2/TaskManager"
        }
        self.SMM_name = "agent://SMM" # name of SMM
        self.cur_robot_pose = {}
        self.AMR_LIFT_IDs = ["AMR_LIFT1", "AMR_LIFT2"]
        self.AMR_TOW_IDs = ["AMR_TOW1", "AMR_TOW2"]
        self.AMR_IDs = ["AMR_LIFT1", "AMR_LIFT2", "AMR_TOW1", "AMR_TOW2"]
        self.robot_goal = {}
        self.Goal_Status = {}
        
    def on_start(self):
        self.ltm = NavigationControlerDataSource("tcp://172.16.165.204:61316")
        self.ltm.connect("tcp://172.16.165.204:61316", "ds://www.arbi.com/Local/NavigationController", 2)

        time.sleep(1)

        Thread(target=self.Goal_check, args=(), daemon=True).start()
    
    def on_data(self, sender, data):
        print("on data from " + sender + " notification : " + data)
        temp_gl = GLFactory.new_gl_from_gl_string(data)
        if temp_gl.get_name() == "MultiRobotPath":
            multi_robot_path = {}
            temp_gl_robot_num = temp_gl.get_expression_size()
            for i in range(temp_gl_robot_num):
                temp_robot_path = temp_gl.get_expression(i).as_generalized_list()
                temp_robot_id = temp_robot_path.get_expression(0)

                temp_gl_path = temp_robot_path.get_expression(1).as_generalized_list()
                temp_gl_path_size = temp_gl_path.get_expression_size()
                for j in range(temp_gl_path_size):
                    temp_path = []
                    temp_path.append(temp_gl_path.get_expression(j))

                multi_robot_path[temp_robot_id] = temp_path
            
            self.ltm.NC.get_multipath_plan(multi_robot_path)

    def on_notify(self, sender, notification):
        print("on notify from " + sender + " notification : " + notification)
        temp_gl = GLFactory.new_gl_from_gl_string(notification)
        if temp_gl.get_name() == "MultiRobotPose":
            # temp_robotID = self.ltm.NC.AMR_IDs[temp_gl.get_expression(0).as_value()]
            # temp_vertex_info = temp_gl.get_expression(1).as_generalized_list()
            # temp_vertex_1 = temp_vertex_info.get_expression(0).as_value()
            # temp_vertex_2 = temp_vertex_info.get_expression(1).as_value()

            # temp_robot_pose = {}
            # temp_robot_pose[temp_robotID] = [temp_vertex_1, temp_vertex_2]
            # self.cur_robot_pose[temp_robotID] = temp_robot_pose[temp_robotID]
            # self.ltm.NC.update_robot_TM(temp_robot_pose)

            robot_num = temp_gl.get_expression_size()
            multi_robot_pose = {}
            for i in range(robot_num):
                robot_gl = temp_gl.get_expression(i).as_generalized_list()
                robot_id = self.AMR_IDs[temp_gl.get_expression(0).as_value()]
                robot_vertex_gl = robot_gl.get_expression(1).as_generalized_list()
                robot_vertex = [robot_vertex_gl.get_expression(0).as_value(), robot_vertex_gl.get_expression(1).as_value()]
                multi_robot_pose[robot_id] = robot_vertex
                self.cur_robot_pose[robot_id] = robot_vertex

            robot_sendTM = self.ltm.NC.update_robot_TM(multi_robot_pose)
            if robot_sendTM:
                self.Control_notify(robot_sendTM, self.robot_goal)
                    
        elif temp_gl.get_name() == "Collidable":
            collide_num = temp_gl.get_expression(0).as_value()
            for i in range(collide_num):
                temp_collide_gl = temp_gl.get_expression(i).as_generalized_list()
                robot_ids = [temp_collide_gl.get_expression(0).as_value().string_value(), temp_collide_gl.get_expression(1).as_value().string_value()]
                self.ltm.NC.update_start_goal_collision(robot_ids)

                for i in range(2):
                    RobotNavCont_gl = "(RobotPath \"" + robot_id + "\" " + robot_path[0] + " " + robot_path[-1] + " "
                    RobotPathPlan_gl = "(RobotPathPlan \"" + robot_id + " "

                    robot_id = robot_ids[i]
                    robot_path = self.ltm.NC.robotTM[robot_id]
                    goal = self.ltm.NC.robotGoal[robot_id]
                    path_gl = "(path"
                    for j in range(len(robot_path)):
                        path_gl += " "
                        path_gl += str(robot_path[j])
                    path_gl += ")"

                    RobotNavCont_gl += path_gl + ")"
                    RobotPathPlan_gl += goal + " " + path_gl + ")"

                    if "LIFT" in robot_id:
                        self.notify(self.LIFT_TM_name[robot_id], RobotNavCont_gl)
                    elif "TOW" in robot_id:
                        self.notify(self.TOW_TM_name[robot_id], RobotNavCont_gl)
                    self.notify(self.SMM_name, RobotPathPlan_gl)
                
                response_gl = self.MultiRobotPath_query(robot_ids)
                self.MultiRobotPath_update(response_gl)
                self.Control_notify(robot_ids)

    def on_request(self, sender, request):
        gl = GLFactory.new_gl_from_gl_string(request)
        if gl.get_name() == "RobotPath":
            task = Thread(target=self.request_path, args=(gl, ))
            task.setDaemon(True)
            task.start()
            return "(ok)"
        else:
            return "(fail)"

    def request_path(self, gl):
        goal = {}
        goal_robot_id = gl.get_expression(0).as_value().string_value()
        goal_vertex = gl.get_expression(2).as_value().int_value()
        goal[goal_robot_id] = goal_vertex
        self.robot_goal[goal_robot_id] = goal_vertex
        robot_id_replan, robot_id_TM = self.ltm.NC.allocate_goal(goal, self.cur_robot_pose)

        for robot_id in robot_id_TM:
            # TODO pause GL 생각하기
            self.send(robot_id, (""))

        self.Control_notify(robot_id_TM, goal)

        if robot_id_replan:
            response = self.MultiRobotPath_query(robot_id_replan)
            response_gl = GLFactory.new_gl_from_gl_string(response)
            self.MultiRobotPath_update(response_gl)

    def Control_notify(self, robot_ids, goal):
        for robot_id in robot_ids:
            RobotNavCont_gl = "(RobotPath \"" + robot_id + "\" "
            RobotPathPlan_gl = "(RobotPathPlan \"" + robot_id + "\" "
            path_gl = "(path"

            robot_paths = self.ltm.NC.robotTM[robot_id]

            RobotNavCont_gl += robot_paths[0] + " " + robot_paths[-1] + " "

            for robot_path in robot_paths:
                path_gl += " "
                path_gl += str(robot_path)
            path_gl += ")"

            RobotNavCont_gl += path_gl + ")"

            RobotPathPlan_gl += goal[robot_id] + " " + path_gl + ")"

            if "LIFT" in robot_id:
                self.send(self.LIFT_TM_name[robot_id], RobotNavCont_gl)
            elif "TOW" in robot_id:
                self.send(self.TOW_TM_name[robot_id], RobotNavCont_gl)

            self.send(self.SMM_name, RobotPathPlan_gl)

    def Goal_check(self):
        while True:
            time.sleep(1)
            for robot_id in self.AMR_IDs:
                goal_end_index = self.ltm.NC.Flag_terminate[robot_id]
                if goal_end_index == 0:
                    self.Goal_Status[robot_id] = True
                else:
                    self.Goal_Status[robot_id] = False

    def MultiRobotPath_update(self, response_gl):
        multipaths = {}
        path_size = response_gl.get_expression_size()
        for i in range(path_size):
            robot_info = response_gl.get_expression(i).as_generalized_list()
            robot_id = robot_info.get_expression(0).as_value()
            path_gl = robot_info.get_expression(1).as_generalized_list()
            path_size = path_gl.get_expression_size()
            path = []
            for j in range(path_size):
                path.append(path_gl.get_expression(j).as_value())
            multipaths[robot_id] = path
        self.ltm.NC.get_multipath_plan(multipaths)
            
    def MultiRobotPath_query(self, robot_id_replan):
        query_gl = "(MultiRobotPath"
        query_block = " (RobotPath {robot_id} {start} {goal})"
        start_id = self.ltm.NC.robotStart
        goal_id = self.ltm.NC.robotGoal
        if "LIFT" in robot_id_replan[0]:
            for robot_id in self.AMR_LIFT_IDs:
                query_gl += query_block.format(robot_id, start_id[robot_id], goal_id[robot_id])
                TM_name = self.LIFT_TM_name[robot_id]
        elif "TOW" in robot_id_replan[0]:
            for robot_id in self.AMR_TOW_IDs:
                query_gl += query_block.format(robot_id, start_id[robot_id], goal_id[robot_id])
                TM_name = self.TOW_TM_name[robot_id]
        query_gl += ")"
        
        response = self.query(TM_name, query_gl)
        return response


if __name__ == "__main__":
    agent = NavigationControlerAgent()
    arbi_agent_excutor.execute(broker_url="tcp://172.16.165.204:61316",
                               agent_name="agent://www.arbi.com/Local/NavigationController",
                               agent=agent, broker_type=2)  # same role with agent.initialize
    while True:
        pass
