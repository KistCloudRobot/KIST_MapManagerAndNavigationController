import sys
import time
import os
import pathlib
import copy
from threading import Condition, Thread
sys.path.append("D:\CloudRobot\Python-mcArbiFramework")
from arbi_agent.agent.arbi_agent import ArbiAgent
from arbi_agent.ltm.data_source import DataSource
from arbi_agent.agent import arbi_agent_excutor
from arbi_agent.model import generalized_list_factory as GLFactory

sys.path.append(str(pathlib.Path(__file__).parent.parent.resolve()))

from MapManagement.MapMOS import MapMOS
from NavigationControl.NavigationControl import NavigationControl

agent_MAPF = "agent://www.arbi.com/Local/MultiAgentPathFinder"
broker_url = "tcp://127.0.0.1:61316"
# broker_url = 'tcp://' + os.environ["JMS_BROKER"]


class NavigationControlerDataSource(DataSource):
    def __init__(self, broker_url):
        
        self.broker = broker_url
        self.connect(self.broker, "ds://www.arbi.com/Local/NavigationController", 2)

        self.map_file = str(pathlib.Path(__file__).parent.parent.resolve()) + "/data/map_cloud.txt"
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
        self.ltm = None
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
        self.TM_name = {
            "AMR_LIFT1": "agent://www.arbi.com/Lift1/TaskManager",
            "AMR_LIFT2": "agent://www.arbi.com/Lift2/TaskManager",
            "AMR_TOW1": "agent://www.arbi.com/Tow1/TaskManager",
            "AMR_TOW2": "agent://www.arbi.com/Tow2/TaskManager"
        }
        self.BI_name = {
            "AMR_LIFT1": "agent://www.arbi.com/Lift1/BehaviorInterface",
            "AMR_LIFT2": "agent://www.arbi.com/Lift2/BehaviorInterface",
            "AMR_TOW1": "agent://www.arbi.com/Tow1/BehaviorInterface",
            "AMR_TOW2": "agent://www.arbi.com/Tow2/BehaviorInterface"
        }
        self.TM_actionID = {
            "AMR_LIFT1": ["\"1\"", "\"5\""],
            "AMR_LIFT2": ["\"2\"", "\"6\""],
            "AMR_TOW1": ["\"3\"", "\"7\""],
            "AMR_TOW2": ["\"4\"", "\"8\""]
        }
        self.SMM_name = "agent://www.arbi.com/Local/MapManager" # name of SMM
        self.AMR_LIFT_IDs = ["AMR_LIFT1", "AMR_LIFT2"]
        self.AMR_TOW_IDs = ["AMR_TOW1", "AMR_TOW2"]
        self.AMR_IDs = ["AMR_LIFT1", "AMR_LIFT2", "AMR_TOW1", "AMR_TOW2"]
        self.cur_robot_pose = {}
        self.robot_goal = {}
        self.move_flag = {
            "AMR_LIFT1": False,
            "AMR_LIFT2": False,
            "AMR_TOW1": False,
            "AMR_TOW2": False
        }
        self.avoid_flag = {
            "AMR_LIFT1": False,
            "AMR_LIFT2": False,
            "AMR_TOW1": False,
            "AMR_TOW2": False
        }
        
    def on_start(self):
        self.ltm = NavigationControlerDataSource(broker_url)
        self.ltm.connect(broker_url, "ds://www.arbi.com/Local/NavigationController", 2)

        time.sleep(1)

        Thread(target=self.Goal_check, args=(), daemon=True).start()

    def on_data(self, sender: str, data: str):
        print(data)

    def on_notify(self, sender, notification):
        print("on notify! " + notification)
        temp_gl = GLFactory.new_gl_from_gl_string(notification)
        gl_name = temp_gl.get_name()
        result = (gl_name == "MultiRobotPose")
        if result:
            robot_num = temp_gl.get_expression_size()
            multi_robot_pose = {}

            for i in range(robot_num):
                robot_gl = temp_gl.get_expression(i).as_generalized_list()
                robot_id = robot_gl.get_expression(0).as_value().string_value()
                robot_vertex_gl = robot_gl.get_expression(1).as_generalized_list()
                robot_vertex = [robot_vertex_gl.get_expression(0).as_value().int_value(),
                                robot_vertex_gl.get_expression(1).as_value().int_value()]
                multi_robot_pose[robot_id] = robot_vertex
                self.cur_robot_pose[robot_id] = robot_vertex

            robot_sendTM = self.ltm.NC.update_robot_TM(multi_robot_pose)

            if robot_sendTM:
                for robot_id in robot_sendTM:
                    if not self.avoid_flag[robot_id]:
                        print("Control_request -> POSE")
                        Thread(target=self.Control_request, args=(robot_id, ), daemon=True).start()
                   
        elif gl_name == "Collidable":
            collide_num = temp_gl.get_expression(0).as_value().int_value()
            for i in range(collide_num):
                temp_collide_gl = temp_gl.get_expression(i+1).as_generalized_list()
                robot_ids = [temp_collide_gl.get_expression(0).as_value().string_value(), temp_collide_gl.get_expression(1).as_value().string_value()]
                self.ltm.NC.update_start_goal_collision(robot_ids)
                
                response = self.MultiRobotPath_query(robot_ids)
                response_gl = GLFactory.new_gl_from_gl_string(response)
                self.MultiRobotPath_update(response_gl)
                for robot_id in robot_ids:
                    if not self.avoid_flag[robot_id]:
                        print("Control_request -> COLLIDABLE")
                        Thread(target=self.Control_request, args=(robot_id, ), daemon=True).start()

    def on_request(self, sender, request):
        print("on request : " + request)
        temp_gl = GLFactory.new_gl_from_gl_string(request)

        if temp_gl.get_name() == "Move":
            robot_goal = {}
            temp_action_id = temp_gl.get_expression(0).as_generalized_list().get_expression(0).as_value().string_value()
            temp_robot_id = temp_gl.get_expression(1).as_value().string_value()
            temp_start = temp_gl.get_expression(2).as_value().int_value()
            temp_goal = temp_gl.get_expression(3).as_value().int_value()
            robot_goal[temp_robot_id] = temp_goal
            self.robot_goal[temp_robot_id] = temp_goal
            robot_id_replan, robot_id_TM = self.ltm.NC.allocate_goal(robot_goal, self.cur_robot_pose)
            robot_id_replan = copy.deepcopy(robot_id_replan)
            print("TM : ", robot_id_TM)
            print("replan : ", robot_id_replan)
            if robot_id_TM:
                for robot_id in robot_id_TM:
                    print("Control_request -> REQUEST TM")
                    Thread(target=self.Control_request, args=(robot_id, ), daemon=True).start()
                    # self.Control_request(robot_id)

            if robot_id_replan:
                response = self.MultiRobotPath_query(robot_id_replan)
                response_gl = GLFactory.new_gl_from_gl_string(response)
                self.MultiRobotPath_update(response_gl)
                for robot_id in robot_id_replan:
                    print("Control_request -> REQUEST REPLAN")
                    Thread(target=self.Control_request, args=(robot_id, ), daemon=True).start()

            goal_request_result = "(MoveResult (actionID {actionID}) {robotID} {result})".format(
                actionID=temp_action_id,
                robotID=temp_robot_id,
                result="okay")

            return goal_request_result
        else:
            print("what?", str(temp_gl))
            return "(fail)"

    def SMM_notify(self, robot_id):
        temp_SMM_gl = "(RobotPathPlan \"{robot_id}\" {goal} {path})"
        path_gl = self.path_gl_generator(self.ltm.NC.robotTM[robot_id])
        SMM_gl = temp_SMM_gl.format(
            robot_id=robot_id,
            goal=self.robot_goal[robot_id],
            path=path_gl)
        self.notify(self.SMM_name, SMM_gl)

    def Control_request(self, robot_id):
        print("control start", robot_id)
        if self.ltm.NC.robotTM[robot_id] and (not (self.cur_robot_pose[robot_id][0] == self.cur_robot_pose[robot_id][1] == self.ltm.NC.robotTM[robot_id][0] and len(self.ltm.NC.robotTM[robot_id]) == 1)):
            if len(self.ltm.NC.robotTM_set[robot_id]) == 1:

                if self.move_flag[robot_id]:
                    temp_CancelTM_gl = "(cancelMove (actionID {actionID}))"
                    CancelTM_gl = temp_CancelTM_gl.format(actionID=self.TM_actionID[robot_id][1])
                    response = self.request(self.BI_name[robot_id], CancelTM_gl)
                    response_gl = GLFactory.new_gl_from_gl_string(response)
                    result = response_gl.get_expression(1).as_value().string_value()
                    print("Cancel {robot_id} path : {result}".format(
                        robot_id=robot_id,
                        result=str(result)))

                self.move_flag[robot_id] = True

                # Request new path
                temp_MoveTM_gl = "(move (actionID {actionID}) {path})"
                # if self.robot_path != self.ltm.NC.robotTM[robot_id]:
                #     self.robot_path = self.ltm.NC.robotTM[robot_id]
                #     path_gl = self.path_gl_generator(self.robot_path)
                path_gl = self.path_gl_generator(self.ltm.NC.robotTM[robot_id])

                MoveTM_gl = temp_MoveTM_gl.format(
                    actionID=self.TM_actionID[robot_id][0], 
                    path=path_gl)
                print("Request {robot_id} to go {next_vertex}".format(
                    robot_id=robot_id,
                    next_vertex=path_gl))

                self.SMM_notify(robot_id)
                response = self.request(self.BI_name[robot_id], MoveTM_gl)
                response_gl = GLFactory.new_gl_from_gl_string(response)
                result = response_gl.get_expression(1).as_value().string_value()
                print("Request {robot_id} to go {next_vertex} : {result}".format(
                    robot_id=robot_id,
                    next_vertex=path_gl,
                    result=result))

            elif len(self.ltm.NC.robotTM_set[robot_id]) >= 2:
                self.avoid_flag[robot_id] = True
                robotTM_set = self.ltm.NC.robotTM_set
                robotTM_scond = self.ltm.NC.robotTM_scond
                # cancel path if the robot is moving
                if self.move_flag[robot_id]:
                    temp_CancelTM_gl = "(cancelMove (actionID {actionID}))"
                    CancelTM_gl = temp_CancelTM_gl.format(
                        actionID=self.TM_actionID[robot_id][1])
                    response = self.request(self.BI_name[robot_id], CancelTM_gl)
                    response_gl = GLFactory.new_gl_from_gl_string(response)
                    result = response_gl.get_expression(
                        1).as_value().string_value()
                    print("Cancel {robot_id} path : {result}".format(
                        robot_id=robot_id,
                        result=str(result)))

                # for path_idx in range(len(self.ltm.NC.robotTM_set[robot_id])):
                for path_idx in range(len(robotTM_set[robot_id])):
                    # waiting
                    # if self.ltm.NC.robotTM_scond[robot_id][path_idx]:
                    if robotTM_scond[robot_id][path_idx]:
                        while True:
                            # counterpart_id = self.ltm.NC.robotTM_scond[robot_id][path_idx][0]
                            # counterpart_idx = self.ltm.NC.robotTM_scond[robot_id][path_idx][1]
                            # counterpart_pose = self.cur_robot_pose[counterpart_id]
                            # counterpart_set = self.ltm.NC.robotTM_set[counterpart_id]
                            counterpart_id = robotTM_scond[robot_id][path_idx][0][0]
                            counterpart_idx = robotTM_scond[robot_id][path_idx][0][1]
                            counterpart_pose = self.cur_robot_pose[counterpart_id]
                            counterpart_set = robotTM_set[counterpart_id]
                            counterpart_check = (counterpart_pose[0] == counterpart_set[counterpart_idx[0]][counterpart_idx[1]]) and (
                                counterpart_pose[1] == counterpart_set[counterpart_idx[0]][counterpart_idx[1]])
                            # update_check = (
                            #     self.ltm.NC.robotTM[robot_id] == self.ltm.NC.robotTM_set[robot_id][path_idx])
                            # update_check = (self.ltm.NC.robotTM[robot_id] == robotTM_set[robot_id][path_idx])
                            # if (counterpart_check and update_check):
                            if counterpart_check:
                                print(robot_id, "Next Path Segment")
                                break
                            time.sleep(1)

                    temp_MoveTM_gl = "(move (actionID {actionID}) {path})"
                    # robot_path = self.ltm.NC.robotTM_set[robot_id][path_idx]
                    robot_path = robotTM_set[robot_id][path_idx]
                    path_gl = self.path_gl_generator(robot_path)

                    MoveTM_gl = temp_MoveTM_gl.format(
                        actionID=self.TM_actionID[robot_id][0],
                        path=path_gl)
                    print("Request {robot_id} to go {next_vertex}".format(
                        robot_id=robot_id,
                        next_vertex=path_gl))
                    self.SMM_notify(robot_id)
                    response = self.request(self.BI_name[robot_id], MoveTM_gl)
                    response_gl = GLFactory.new_gl_from_gl_string(response)
                    result = response_gl.get_expression(1).as_value().string_value()
                    print("Request {robot_id} to go {next_vertex} : {result}".format(
                        robot_id=robot_id,
                        next_vertex=path_gl,
                        result=result))
                self.avoid_flag[robot_id] = False
        else:
            print(robot_id, " no path")
            self.SMM_notify(robot_id)

    def path_gl_generator(self, path):
        path_gl = "(path"
        for path_i in path:
            path_gl += " "
            path_gl += str(path_i)
        path_gl += ")"

        return path_gl

    def Goal_check(self):
        while True:
            time.sleep(1)
            for robot_id in self.AMR_IDs:
                goal_end_index = self.ltm.NC.Flag_terminate[robot_id]

                if self.move_flag[robot_id] and (goal_end_index == 0):
                    self.move_flag[robot_id] = False
                    goal_result_gl = "(MoveResult (actionID \"{actionID}\") \"{robotID}\" \"{result}\")".format(
                        actionID=self.TM_actionID[robot_id],
                        robotID=robot_id,
                        result="success")
                    self.send(self.TM_name[robot_id], goal_result_gl)
                else:
                    pass

    def MultiRobotPath_update(self, response_gl):
        multipaths = dict()
        path_size = response_gl.get_expression_size()
        robot_pose = {}
        for i in range(path_size):
            robot_info = response_gl.get_expression(i).as_generalized_list()
            robot_id = robot_info.get_expression(0).as_value().string_value()
            path_gl = robot_info.get_expression(1).as_generalized_list()
            path_size = path_gl.get_expression_size()
            path = list()
            for j in range(path_size):
                path.append(path_gl.get_expression(j).as_value().int_value())
            multipaths[robot_id] = path
            robot_pose[robot_id] = self.cur_robot_pose[robot_id]
        self.ltm.NC.get_multipath_plan(multipaths)
        # self.ltm.NC.update_robot_TM(self.cur_robot_pose)
        self.ltm.NC.update_robot_TM(robot_pose)
            
    def MultiRobotPath_query(self, robot_id_replan):
        request_gl = "(MultiRobotPath"

        for robot_id in robot_id_replan:
            # start_id = self.ltm.NC.robotStart[robot_id]
            start_id = self.cur_robot_pose[robot_id][0]
            goal_id = self.ltm.NC.robotGoal[robot_id]
            if goal_id == -1:
                goal_id = start_id

            request_gl += " (RobotPath \"" + robot_id + "\" " + str(start_id) + " " + str(goal_id) + ")"

            if len(robot_id_replan) == 1:
                counterpart_check = {0: 1, 1: 0, 2: 3, 3: 2}
                robot_index = self.AMR_IDs.index(robot_id)
                c_robot_id = self.AMR_IDs[counterpart_check[robot_index]]
                c_start_id = self.cur_robot_pose[c_robot_id][0]
                c_goal_id = self.cur_robot_pose[c_robot_id][1]
                request_gl += " (RobotPath \"" + c_robot_id + "\" " + str(c_start_id) + " " + str(c_goal_id) + ")"

        request_gl += ")"

        time.sleep(0.05)
        print("robot path request :", request_gl)
        response = self.request(agent_MAPF, request_gl)
        print("robot path response :", response)

        return response


if __name__ == "__main__":
    agent = NavigationControlerAgent()
    arbi_agent_excutor.execute(broker_url=broker_url,
                               agent_name="agent://www.arbi.com/Local/NavigationController",
                               agent=agent, broker_type=2)  # same role with agent.initialize
    while True:
        pass
