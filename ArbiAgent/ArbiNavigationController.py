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
from NavigationControl.NavigationControl import NavigationControl
from DataType.RobotInfo import RobotInfo
from DataType.CallInfo import CallInfo

class NavigationControlerDataSource(DataSource):
    def __init__(self, broker_url):
        
        self.broker = broker_url
        self.connect(self.broker, "", BrokerType.ZERO_MQ)

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
        self.LIFT_TM_name = "agent://LIFT_TM" # name of LIFT-TM
        self.TOW_TM_name = "agent://TOW_TM" # name of TOW-TM
        self.SMM_name = "agent://SMM" # name of SMM
        self.cur_robot_pose = {}
        self.AMR_LIFT_IDs = ["AMR_LIFT1", "AMR_LIFT2"]
        self.AMR_TOW_IDs = ["AMR_TOW1", "AMR_TOW2"]
        self.AMR_IDs = ["AMR_LIFT1", "AMR_LIFT2", "AMR_TOW1", "AMR_TOW2"]
        self.robot_goal = {}
        self.Goal_Status = {}
        
    def on_start(self):
        self.ltm = NavigationControlerDataSource("tcp://127.0.0.1:61616")
        self.ltm.connect("tcp://127.0.0.1:61616", "ds://www.arbi.com/NavigationControllerAgent", BrokerType.ZERO_MQ)

        time.sleep(1)

        self.Goal_check()
    
    def on_data(self, sender, data):
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
            RobotNavCont_gl = "(RobotNavCont {robot_id} {type} {path})"
            RobotPathPlan_gl = "(RobotPathPlan {robot_id} {goal} {path})"  
            path_gl = "(path"

            collide_num = temp_gl.get_expression(0).as_value()
            for i in range(collide_num):
                temp_collide_gl = temp_gl.get_expression(i).as_generalized_list()
                robot_ids = [temp_collide_gl.get_expression(0).as_value(), temp_collide_gl.get_expression(1).as_value()]
                self.ltm.NC.update_start_goal_collision(robot_ids)

                for i in range(2):
                    robot_id = robot_ids[i]
                    robot_path = self.ltm.NC.robotTM[robot_id]
                    for j in range(len(robot_path)):
                        path_gl += " "
                        path_gl += str(robot_path[j])
                    path_gl += ")"
                    goal = self.ltm.NC.robotGoal[robot_id]
                    if "LIFT" in robot_ids[i]:
                        self.notify(self.LIFT_TM_name, RobotNavCont_gl.format(robot_id, "path", path_gl))
                        self.notify(self.SMM_name, RobotPathPlan_gl.format(robot_id, goal, path_gl))
                    elif "TOW" in robot_ids[i]:
                        self.notify(self.TOW_TM_name, RobotNavCont_gl.format(robot_id, "path", path_gl))
                        self.notify(self.SMM_name, RobotPathPlan_gl.format(robot_id, goal, path_gl))
                
                response_gl = self.MultiRobotPath_query(robot_ids)
                self.MultiRobotPath_update(response_gl)
                self.Control_notify(robot_ids)

    def on_request(self, sender, request):
        temp_gl = GLFactory.new_gl_from_gl_string(request)

        if temp_gl.get_name() == "goal": # from Robot TM
            ### query of goal Not Assigned ###
            ### response of goal Not Assigned ###
            goal = {}
            goal_robot_id = temp_gl.get_expression() ### TEMP ###
            goal_vertex = temp_gl.get_expression() ### TEMP ###
            goal[goal_robot_id] = goal_vertex
            self.robot_goal[goal_robot_id] = goal_vertex
            robot_id_replan, robot_id_TM = self.ltm.NC.allocate_goal(goal, self.cur_robot_pose)

            self.Control_notify(robot_id_TM, goal)

            if robot_id_replan:
                response_gl = self.MultiRobotPath_query(robot_id_replan)
                self.MultiRobotPath_update(response_gl)
        
        while not self.Goal_Status[goal_robot_id]:
            if self.Goal_Status[goal_robot_id]:
                break

        if self.Goal_Status[goal_robot_id]:
            return [] ### TEMP ###

    def Control_notify(self, robot_ids, goal):
        RobotNavCont_gl = "(RobotNavCont {robot_id} {type} {path})"
        RobotPathPlan_gl = "(RobotPathPlan {robot_id} {goal} {path})"  
        path_gl = "(path"
        for robot_id in robot_ids:
            robot_paths = self.ltm.NC.robotTM[robot_id]
            for robot_path in robot_paths:
                path_gl += " "
                path_gl += str(robot_path)
            path_gl += ")"

            if "LIFT" in robot_id:
                self.notify(self.LIFT_TM_name, RobotNavCont_gl.format(robot_id, "path", path_gl))
                self.notify(self.SMM_name, RobotPathPlan_gl.format(robot_id, goal[robot_id], path_gl))
            elif "TOW" in robot_id:
                self.notify(self.TOW_TM_name, RobotNavCont_gl.format(robot_id, "path", path_gl))
                self.notify(self.SMM_name, RobotPathPlan_gl.format(robot_id, goal[robot_id, path_gl]))

    def Goal_check(self):
        for robot_id in self.AMR_IDs:
            goal_end_index = self.ltm.NC.Flag_terminate[robot_id]
            if goal_end_index == 0:
                self.Goal_Status[robot_id] = True
            else:
                self.Goal_Status[robot_id] = False
        
        threading.Timer(1, self.Goal_check).start()

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
                TM_name = self.LIFT_TM_name
        elif "TOW" in robot_id_replan[0]:
            for robot_id in self.AMR_TOW_IDs:
                query_gl += query_block.format(robot_id, start_id[robot_id], goal_id[robot_id])
                TM_name = self.TOW_TM_name
        query_gl += ")"
        
        response = self.query(TM_name, query_gl)
        return response

if __name__ == "__main__":
    agent = NavigationControlerAgent()
    arbi_agent_excutor.execute(broker_url="tcp://127.0.0.1:61616", agent_name="agent://www.arbi.com/NavigationControllerAgent",
                               agent=agent, broker_type=2)  # same role with agent.initialize
