import sys
import threading
import time
import os
import pathlib
import copy
from threading import Condition, Thread

sys.path.append("/home/kist/demo/src/Python_mcArbiFramework/")


from arbi_agent.agent.arbi_agent import ArbiAgent
from arbi_agent.ltm.data_source import DataSource
from arbi_agent.agent import arbi_agent_executor
from arbi_agent.model import generalized_list_factory as GLFactory

from MapManagement.MapMOS import MapMOS
from NavigationControl.NavigationControl import NavigationControl

agent_MAPF = "agent://www.arbi.com/Local/MultiAgentPathFinder"
broker_url = "tcp://192.168.0.14:61313"


# broker_url = 'tcp://' + os.environ["JMS_BROKER"]

class NavigationControllerDataSource(DataSource):
    def __init__(self, broker_url):

        self.broker = broker_url  # broker address
        self.connect(self.broker, "ds://www.arbi.com/Local/NavigationController", 2)  # connect broker

        self.map_file = str(pathlib.Path(__file__).parent.parent.resolve()) + "/data/map_cloud.txt"  # load map file (~/data/map_cloud.txt)
        self.MAP = MapMOS(self.map_file)  # interprete map

        self.AMR_IDs = ["AMR_LIFT1", "AMR_LIFT2", "AMR_TOW1", "AMR_TOW2"]  # AMR IDs
        self.AMR_LIFT_IDs = list()  # LIFT IDs
        self.AMR_TOW_IDs = list()  # TOW IDs
        for identifier in self.AMR_IDs:
            if "LIFT" in identifier:
                self.AMR_LIFT_IDs.append(identifier)
            elif "TOW" in identifier:
                self.AMR_TOW_IDs.append(identifier)

        self.NC = NavigationControl(self.AMR_IDs, self.AMR_LIFT_IDs,
                                    self.AMR_TOW_IDs)  # launch NC(NavigationController)


class NavigationControllerAgent(ArbiAgent):
    def __init__(self):
        super().__init__()
        self.ltm = None  # ltm address (to be modified after on_start)
        self.lock = Condition()

        self.CM_name = "agent://www.arbi.com/Local/ContextManager"

        self.TM_name = {"AMR_LIFT1": "agent://www.arbi.com/Lift1/TaskManager",
                        "AMR_LIFT2": "agent://www.arbi.com/Lift2/TaskManager",
                        "AMR_TOW1": "agent://www.arbi.com/Tow1/TaskManager",
                        "AMR_TOW2": "agent://www.arbi.com/Tow2/TaskManager"}  # agent address of robotTaskManager(robotTM)

        self.BI_name = {
            "AMR_LIFT1": "agent://www.arbi.com/Lift1/BehaviorInterface",
            "AMR_LIFT2": "agent://www.arbi.com/Lift2/BehaviorInterface",
            "AMR_TOW1": "agent://www.arbi.com/Tow1/BehaviorInterface",
            "AMR_TOW2": "agent://www.arbi.com/Tow2/BehaviorInterface"
        }  # agent adress of robotBehaviorInterface(robotBI)

        self.SMM_name = "agent://www.arbi.com/Local/MapManager"  # agent address of SemanticMapManager(SMM)

        self.BI_actionID = {
            "AMR_LIFT1": ["\"1\"", "\"5\""],
            "AMR_LIFT2": ["\"2\"", "\"6\""],
            "AMR_TOW1": ["\"3\"", "\"7\""],
            "AMR_TOW2": ["\"4\"", "\"8\""]
        }  # actionID [moveID, cancelID] to send to robotBI

        self.AMR_IDs = ["AMR_LIFT1", "AMR_LIFT2", "AMR_TOW1", "AMR_TOW2"]  # AMR IDs
        self.AMR_LIFT_IDs = list()  # LIFT IDs
        self.AMR_TOW_IDs = list()  # TOW IDs
        for identifier in self.AMR_IDs:
            if "LIFT" in identifier:
                self.AMR_LIFT_IDs.append(identifier)
            elif "TOW" in identifier:
                self.AMR_TOW_IDs.append(identifier)

        self.cur_robot_pose = {}  # current pose of robot (notify from SMM) e.g. {robotID: [vertex, vertex]}
        self.goal_actionID = {}  # actionID from TM(goal request) e.g. {robotID: actionID}
        self.real_goal = {
            "AMR_LIFT1": -1,
            "AMR_LIFT2": -1,
            "AMR_TOW1": -1,
            "AMR_TOW2": -1
        }
        self.actual_goal = {
            "AMR_LIFT1": -1,
            "AMR_LIFT2": -1,
            "AMR_TOW1": -1,
            "AMR_TOW2": -1
        }
        self.move_flag = {
            "AMR_LIFT1": False,
            "AMR_LIFT2": False,
            "AMR_TOW1": False,
            "AMR_TOW2": False
        }  # identify whether robot is moving
        self.avoid_flag = {
            "AMR_LIFT1": False,
            "AMR_LIFT2": False,
            "AMR_TOW1": False,
            "AMR_TOW2": False
        }  # identify whether robot is in avoiding plan
        self.collide_flag = {
            "AMR_LIFT1": False,
            "AMR_LIFT2": False,
            "AMR_TOW1": False,
            "AMR_TOW2": False
        }  # identify whether robot is in avoiding plan

        self.thread_flag = {
            "AMR_LIFT1": False,
            "AMR_LIFT2": False,
            "AMR_TOW1": False,
            "AMR_TOW2": False
        }

        self.lock = {
            "AMR_LIFT1": threading.Condition(),
            "AMR_LIFT2": threading.Condition(),
            "AMR_TOW1": threading.Condition(),
            "AMR_TOW2": threading.Condition()
        }

        self.count = 0

    def on_start(self):  # executed when the agent initializes
        self.ltm = NavigationControllerDataSource(broker_url)  # ltm class
        self.ltm.connect(broker_url, "ds://www.arbi.com/Local/NavigationController", 2)  # connect ltm

        time.sleep(1)

        Thread(target=self.Goal_check, args=(), daemon=True).start()  # chehck goal status of each robot every 1sec

    def on_data(self, sender: str, data: str):  # N/A
        # print(data)
        pass

    def on_notify(self, sender, notification):  # executed when the agent gets notification
        # print("[on Notify] " + notification)
        temp_gl = GLFactory.new_gl_from_gl_string(notification)  # notification to gl
        gl_name = temp_gl.get_name()  # get name of gl
        
        if gl_name == "MultiRobotPose":  # "MultiRobotPose" update from SMM
            ''' MultiRobotPose gl format: (MultiRobotPose (RobotPose $robot_id (vertex $vertex_id $vertex_id)), ...) '''

            robot_num = temp_gl.get_expression_size()  # check updated number of robots
            multi_robot_pose = {}  # {robotID: [vertex, vertex]}

            for i in range(robot_num):  # update robot pose from gl
                '''RobotPose_gl format: (RobotPose $robot_id (vertex $vertex_id $vertex_id))'''

                RobotPose_gl = temp_gl.get_expression(i).as_generalized_list()  # get ith gl content
                robot_id = RobotPose_gl.get_expression(0).as_value().string_value()  # get robotID
                robot_vertex_gl = RobotPose_gl.get_expression(1).as_generalized_list()  # get current vertex
                robot_vertex = [robot_vertex_gl.get_expression(0).as_value().int_value(),
                                robot_vertex_gl.get_expression(1).as_value().int_value()]  # current pose to list
                multi_robot_pose[robot_id] = robot_vertex
                self.cur_robot_pose[robot_id] = robot_vertex  # update current robot pose in agent

            robot_id_BI = self.ltm.NC.update_robot_TM(multi_robot_pose)  # update robot pose in NC
            ''' robot_sendTM: robotID that has any change of its path and notify the information of robotID to robotBI '''

            if robot_id_BI:  # check whether robot_id_BI is empty
                print("3Thread create by MultiRobotPose num : " + str(len(robot_id_BI)))
                for robot_id in robot_id_BI:
                    self.SMM_notify(robot_id)  # # notify SMM of same control information
                    # if not self.avoid_flag[robot_id]:
                    if (not self.avoid_flag[robot_id]) and (not self.move_flag[robot_id]):
                        print("[INFO] {RobotID} cancel move".format(RobotID=robot_id))
                        self.cancel_move(robot_id)
                        if self.thread_flag[robot_id]:
                            self.thread_flag[robot_id] = False
                            with self.lock[robot_id]:
                                self.lock[robot_id].wait()
                        print("[INFO] {RobotID} Control request by <COLLIDABLE> Notification [{num}]".format(RobotID=robot_id, num=self.count))
                        Thread(target=self.Control_request, args=(robot_id, True, False, self.count), daemon=True).start()
                        self.count = self.count + 1

        elif gl_name == "Collidable":  # "Collidable" update from SMM
            ''' Collidable gl format: (Collidable $num (pair $robot_id $robot_id $time), …) 
                                    num: number of collidable set
                                    time: in which collision is expected '''

            collide_num = temp_gl.get_expression(0).as_value().int_value()  # get number of collidable set
            for i in range(collide_num):
                ''' collidable_set(pair) gl format: (pair $robot_id $robot_id $time) '''

                collidable_set_gl = temp_gl.get_expression(i + 1).as_generalized_list()  # get ith collidable set
                robot_ids = [collidable_set_gl.get_expression(0).as_value().string_value(),
                             collidable_set_gl.get_expression(
                                 1).as_value().string_value()]  # get robotIDs of collidable set
                check = 0 # check if the robots are already in collidable triggered plan
                for robot_id in robot_ids:
                    check += self.collide_flag[robot_id]
                if check == 0:
                    for robot_id in robot_ids:
                        self.collide_flag[robot_id] = True
                        print("[INFO] {RobotID} cancel move by collidable".format(RobotID=robot_id))
                        self.cancel_move(robot_id)
                    for robot_id in robot_ids:
                        # self.collide_flag[robot_id] = True
                        # print("[INFO] {RobotID} cancel move by collidable".format(RobotID=robot_id))
                        # self.cancel_move(robot_id)
                        if self.thread_flag[robot_id]:
                            self.thread_flag[robot_id] = False
                            with self.lock[robot_id]:
                                self.lock[robot_id].wait()

                    self.ltm.NC.update_start_goal_collision(robot_ids)  # update collidable robotIDs in NC

                    path_response = self.MultiRobotPath_query(robot_ids)  # query about not collidable path to MultiAgentPathFinder(MAPF)
                    time.sleep(0.5)
                    path_response_gl = GLFactory.new_gl_from_gl_string(path_response)
                    ''' path_response gl format: (MultiRobotPath (RobotPath $robot_id (path $v_id1 $v_id2 $v_id3, ...)), …) '''

                    robot_ids = self.MultiRobotPath_update(path_response_gl)  # update path of robot in NC
                    print("4Thread create by collidable : " + str(robot_ids))
                    for robot_id in robot_ids:
                        self.SMM_notify(robot_id)  # # notify SMM of same control information
                        # print("[INFO] {RobotID} cancel move".format(RobotID=robot_id))
                        # self.cancel_move(robot_id)
                        # print("[INFO] {RobotID} Control request by <COLLIDABLE> Notification [{num}]".format(RobotID=robot_id, num=self.count))
                        # Thread(target=self.Control_request, args=(robot_id, True, False, self.count), daemon=True).start()
                        self.count = self.count + 1
                        if self.ltm.NC.robotTM[robot_id]:
                            print("[INFO] {RobotID} Control request by <COLLIDABLE> Notification [{num}]".format(RobotID=robot_id, num=self.count))
                            Thread(target=self.Control_request, args=(robot_id, True, False, self.count), daemon=True).start()  # control request of robotID
                        else:
                            self.move_flag[robot_id] = True

    def on_request(self, sender, request):  # executed when the agent gets request
        print("[on Request] " + request)
        temp_gl = GLFactory.new_gl_from_gl_string(request)

        if temp_gl.get_name() == "Move":  # "Move" request from robotTM
            ''' Move gl format: (Move (actionID $actionID) $robotID $start $end)
                                start: start vertex
                                end: end(goal) vertex '''
            
            robot_goal = {}  # {robotID: goalVertex}
            action_id = temp_gl.get_expression(0).as_generalized_list().get_expression(0).as_value().string_value()  # get actionID
            robot_id = temp_gl.get_expression(1).as_value().string_value()  # get robotID
            requested_robot_id = robot_id
            self.goal_actionID[robot_id] = action_id  # update actionID in agent
            temp_start = temp_gl.get_expression(2).as_value().int_value()
            real_goal = temp_gl.get_expression(3).as_value().int_value()  # get goalVertex
            self.real_goal[robot_id] = real_goal

            actual_goal_gl = "(context (NavigationVertex {realGoal} $vertex))".format(realGoal=real_goal)
            # print("[{action_id}][Query1] Query to CM about vertex {realGoal}".format(action_id=action_id, realGoal=real_goal))
            actual_goal_response = self.query(self.CM_name, actual_goal_gl)
            actual_goal_response_gl = GLFactory.new_gl_from_gl_string(actual_goal_response)
            # print("[{action_id}][Query2] response from CM : {response_gl}".format(action_id=action_id, response_gl=str(actual_goal_response_gl)))
            actual_goal = actual_goal_response_gl.get_expression(0).as_generalized_list().get_expression(1).as_value().int_value()
            # print("[{action_id}] actual goal : {actual_goal}".format(action_id=action_id, actual_goal=actual_goal))
            self.actual_goal[robot_id] = actual_goal

            robot_goal[robot_id] = actual_goal  # robotID-goalVertex pair
            robot_id_replan, robot_id_BI = self.ltm.NC.allocate_goal(robot_goal,self.cur_robot_pose)  # allocate goal in NC
            ''' robot_id_replan: robot which needs to get replanning -> MAPF
                robot_id_BI: robotID that has any change of its path and notify the information of robotID to robotBI '''
            
            if robot_id_BI:  # check whether robot_id_BI is empty
                print("1Thread create num : " + str(len(robot_id_BI)))
                print("1Thread create BI : " , robot_id_BI)
                for robot_id in robot_id_BI:
                    if len(robot_id_replan) < 2:
                        self.SMM_notify(robot_id)  # # notify SMM of same control information
                        print("[{action_id}][INFO2] {RobotID} cancel move".format(action_id=action_id, RobotID=robot_id))
                        self.cancel_move(robot_id)
                        print("[{action_id}][INFO2] {RobotID} cancel move finish".format(action_id=action_id, RobotID=robot_id))
                        print("[{action_id}][INFO2] {RobotID} {flag}".format(action_id=action_id, RobotID=robot_id, flag=str(self.thread_flag[robot_id])))
                        if self.thread_flag[robot_id]:
                            self.thread_flag[robot_id] = False
                            with self.lock[robot_id]:
                                print("[{action_id}][INFO2] {RobotID} waiting thread".format(action_id=action_id, RobotID=robot_id))
                                self.lock[robot_id].wait()
                        print("[{action_id}][INFO2] {RobotID} Control request by <Move> Request [{num}]".format(action_id=action_id, RobotID=robot_id, num=self.count))
                        Thread(target=self.Control_request, args=(robot_id, False, True, self.count, ), daemon=True).start()  # control request of robotID
                        self.count = self.count + 1

            if robot_id_replan:  # check whether robot_id_replan is empty
                for robot_id in robot_id_replan:
                    print("[{action_id}][INFO3] {RobotID} cancel move".format(action_id=action_id, RobotID=robot_id))
                    self.cancel_move(robot_id)
                    print("[{action_id}][INFO3] {RobotID} cancel move finish".format(action_id=action_id,
                                                                                     RobotID=robot_id))

                path_response = self.MultiRobotPath_query(robot_id_replan)  # query about not collidable path to MultiAgentPathFinder(MAPF)
                path_response_gl = GLFactory.new_gl_from_gl_string(path_response)
                ''' path_response_gl gl format: (MultiRobotPath (RobotPath $robot_id (path $v_id1 $v_id2 $v_id3, ...)), …) '''

                print("before update")
                robot_ids = self.MultiRobotPath_update(path_response_gl)  # update path of robot in NC
                print("after update")
                for robot_id in robot_ids:
                    self.SMM_notify(robot_id)
                print("2Thread create num : " + str(robot_ids))
                print(robot_id_replan, 1111111111111111111)
                for robot_id in robot_id_replan:
                    # if self.real_goal[robot_id] != -1:
                    # print("[{action_id}][INFO3] {RobotID} cancel move".format(action_id=action_id, RobotID=robot_id))
                    # self.cancel_move(robot_id)
                    # print("[{action_id}][INFO3] {RobotID} cancel move finish".format(action_id=action_id, RobotID=robot_id))
                    # print("[{action_id}][INFO3] {RobotID} {flag}".format(action_id=action_id, RobotID=robot_id,
                    #                                                      flag=str(self.thread_flag[robot_id])))
                    if self.thread_flag[robot_id]:
                        self.thread_flag[robot_id] = False
                        with self.lock[robot_id]:
                            print("[{action_id}][INFO3] {RobotID} waiting thread".format(action_id=action_id,
                                                                                         RobotID=robot_id))
                            self.lock[robot_id].wait()
                            print("[{action_id}][INFO3] {RobotID} waiting finish".format(action_id=action_id,
                                                                                         RobotID=robot_id))
                    # self.SMM_notify(robot_id)  # # notify SMM of same control information
                    if self.ltm.NC.robotTM[robot_id]:
                        print("[{action_id}][INFO3] {RobotID} Control request by <Move> Request [{num}]".format(action_id=action_id, RobotID=robot_id, num=self.count))
                        Thread(target=self.Control_request, args=(robot_id, False, True, self.count, ), daemon=True).start()  # control request of robotID
                    else:
                        self.move_flag[robot_id] = True
                    self.count = self.count + 1

            goal_request_result = "(ok)"  # response to robotTM that request is successful
            print("[{action_id}][on Request6] Response of request of {RobotID}: {Result}".format(action_id=action_id, RobotID=requested_robot_id, Result=goal_request_result))

            return goal_request_result
        else:
            print("what?", str(temp_gl))
            return "(fail)"

    def cancel_move(self, robot_id):
        if len(self.ltm.NC.robotTM[robot_id]) == 1:  # check whether robot is stationary
            print("[" + str(robot_id) + "]" + str(self.cur_robot_pose[robot_id][0]))
            print("[" + str(robot_id) + "]" + str(self.cur_robot_pose[robot_id][1]))
            stationary_check = (self.cur_robot_pose[robot_id][0] == self.cur_robot_pose[robot_id][1] ==
                                self.ltm.NC.robotTM[robot_id][0])  # True if robot is stationary and will be stationary
        else:
            stationary_check = False

        print("[" + str(robot_id) + "]" + str(self.ltm.NC.robotTM[robot_id]))
        print("[" + str(robot_id) + "]" + str(stationary_check))
        print("[" + str(robot_id) + "]" + str(self.ltm.NC.robotTM_set[robot_id]))
        print("[" + str(robot_id) + "]" + str(len(self.ltm.NC.robotTM_set[robot_id])))
        if self.ltm.NC.robotTM[robot_id] and (not stationary_check):  # check whether robot has path and not stationary
            ''' if robot is avoiding against counterpart robot, path of the robot is split
                e.g. self.ltm.NC.robotTM_set[avoidingRobotID] == [[path], [path]]
                     self.ltm.NC.robotTM_set[counterpartRobotID] == [[path]] '''

            if len(self.ltm.NC.robotTM_set[robot_id]) == 1:  # not split path (not avoiding path)
                ### Cancel current control request if robot is moving ###
                if self.move_flag[robot_id]:  # check whether robot is moving
                    ''' If robot is moving now, current path should be canceled.
                        Cancel actionID:
                            "AMR_LIFT1": "\"5\""
                            "AMR_LIFT2": "\"6\""
                            "AMR_TOW1": "\"7\""
                            "AMR_TOW2": "\"8\""
                        cancelMove gl format: (cancelMove (actionID $actionID)) '''

                    temp_Cancel_gl = "(cancelMove (actionID {actionID}))"
                    Cancel_gl = temp_Cancel_gl.format(actionID=self.BI_actionID[robot_id][1])  # Cancel actionID
                    self.move_flag[robot_id] = False  # update moving state of robot
                    print("[Request CancelMove1]\t{RobotID}".format(RobotID=robot_id))
                    cancel_response = self.request(self.BI_name[robot_id],
                                                   Cancel_gl)  # get response of Cancel request
                    cancel_response_gl = GLFactory.new_gl_from_gl_string(cancel_response)
                    if cancel_response_gl.get_name() == "fail":
                        print("[Response CancelMove1]\t{RobotID}: FAIL".format(RobotID=robot_id))
                    else:
                        result = cancel_response_gl.get_expression(
                            1).as_value().string_value()  # "success" if Cancel request is done
                        print("[Response CancelMove1]\t{RobotID}: {Result}".format(RobotID=robot_id, Result=result))
            elif len(self.ltm.NC.robotTM_set[robot_id]) >= 2:  # split path (avoiding path)
                self.avoid_flag[robot_id] = True  # update avoiding state of robot
                counterpart_check = {0: 1, 1: 0, 2: 3, 3: 2}
                robot_index = self.AMR_IDs.index(robot_id)
                c_robot_id = self.AMR_IDs[counterpart_check[robot_index]]  # get robotID of counterpart robot
                self.avoid_flag[c_robot_id] = True

                ''' self.ltm.NC.robotTM_scond: start condition of split path
                    e.g. AMR_LIFT1 is avoiding robot and AMR_LIFT2 is counterpart robot
                         self.ltm.NC.robotTM_set["AMR_LIFT1"] == [[1, 2, 3], [4, 5, 6, 7]]
                         self.ltm.NC.robotTM_set["AMR_LIFT2"] == [4, 5, 6, 7, 8, 9]
                         self.ltm.NC.robotTM_scond["AMR_LIFT1"] == [[], [["AMR_LIFT2", [0, 1]]]]

                    self.ltm.NC.robotTM_scond["AMR_LIFT1"]: start condition of "AMR_LIFT1"(avoidingRobot)
                    -> no condition of 0th path
                    -> [["AMR_LIFT2", [0, 1]]] condition of 1th path which means start after "AMR_LIFT2" passes 1th element(5) in 0th path([4, 5, 6, 7, 8, 9]) '''

                ### Cancel current control request if robot is moving ###
                if self.move_flag[robot_id]:  # check whether robot is moving
                    temp_Cancel_gl = "(cancelMove (actionID {actionID}))"
                    Cancel_gl = temp_Cancel_gl.format(actionID=self.BI_actionID[robot_id][1])  # Cancel actionID
                    self.move_flag[robot_id] = False  # update moving state of robot
                    print("[Request CancelMove2]\t{RobotID}".format(RobotID=robot_id))
                    cancel_response = self.request(self.BI_name[robot_id],
                                                   Cancel_gl)  # get response of Cancel request
                    cancel_response_gl = GLFactory.new_gl_from_gl_string(cancel_response)
                    if cancel_response_gl.get_name() == "fail":
                        print("[Response CancelMove2]\t{RobotID}: FAIL".format(RobotID=robot_id))
                    else:
                        result = cancel_response_gl.get_expression(1).as_value().string_value()  # "success" if Cancel request is done
                        print("[Response CancelMove2]\t{RobotID}: {Result}".format(RobotID=robot_id, Result=result))

    def Control_request(self, robot_id, collide, replan, count):
        c = "[" + str(count) + "]"
        self.count = self.count + 1
        print(c + "[Info] Control request start : " + robot_id)
        self.thread_flag[robot_id] = True
        self.move_flag[robot_id] = True
        # print("111111111111111", self.ltm.NC.robotTM_set[robot_id])
        ''' if robot is in stationary state, output from MAPF path is same with current vertex
            e.g. "AMR_TOW1" : in stationary state, no plan to move
                -> self.cur_robot_pose["AMR_TOW1"][0] == self.cur_robot_pose["AMR_TOW1"][1]
                -> len(self.ltm.NC.robotTM[robot_id][0]) == 1
            => no control is needed '''
        
        ''' self.ltm.NC.robotTM[robotID]: current path of robotID '''
    
        if collide:
            self.collide_flag[robot_id] = True
            print(111111111111111111111111, robot_id, "COLLIDABLE")
            # counterpart_check = {0: 1, 1: 0, 2: 3, 3: 2}
            # robot_index = self.AMR_IDs.index(robot_id)
            # c_robot_id = self.AMR_IDs[counterpart_check[robot_index]]  # get robotID of counterpart robot
            # self.collide_flag[c_robot_id] = True
            # time.sleep(1)
        elif replan:
            time.sleep(1)

        # if (self.cur_robot_pose[robot_id][0] == self.cur_robot_pose[robot_id][1]) and (self.cur_robot_pose[robot_id][0] == self.actual_goal[robot_id]):
        #     print(c + "current pose equals goal -> thread terminate")
        #     self.thread_flag[robot_id] = False

        while self.thread_flag[robot_id]:
            print(c + "waiting path")
            time.sleep(0.5)
            if self.ltm.NC.robotTM[robot_id]:
                print(c + "got path")
                print(c + " robot_id " + str(robot_id))
                print(c + " ROBOT_TM " + str(self.ltm.NC.robotTM[robot_id]))
                break
            else:
                continue
        
        if self.thread_flag[robot_id]:
            if (len(self.ltm.NC.robotTM[robot_id]) == 1):  # check whether robot is stationary
                stationary_check = (self.cur_robot_pose[robot_id][0] == self.cur_robot_pose[robot_id][1]) and (self.actual_goal[robot_id] == -1)  # True if robot is stationary and will be stationary
            else:
                stationary_check = False

            if self.ltm.NC.robotTM[robot_id] and (not stationary_check):  # check whether robot has path and not stationary
                ''' if robot is avoiding against counterpart robot, path of the robot is split
                    e.g. self.ltm.NC.robotTM_set[avoidingRobotID] == [[path], [path]]
                         self.ltm.NC.robotTM_set[counterpartRobotID] == [[path]] '''

                if len(self.ltm.NC.robotTM_set[robot_id]) == 1:  # not split path (not avoiding path)
                    ### Cancel current control request if robot is moving ###
                    # if self.move_flag[robot_id]:  # check whether robot is moving
                    #     ''' If robot is moving now, current path should be canceled.
                    #         Cancel actionID:
                    #             "AMR_LIFT1": "\"5\""
                    #             "AMR_LIFT2": "\"6\""
                    #             "AMR_TOW1": "\"7\""
                    #             "AMR_TOW2": "\"8\""
                    #         cancelMove gl format: (cancelMove (actionID $actionID)) '''
                    
                    #     while True:
                    #         temp_Cancel_gl = "(cancelMove (actionID {actionID}))"
                    #         Cancel_gl = temp_Cancel_gl.format(actionID=self.BI_actionID[robot_id][1])  # Cancel actionID
                    #         self.move_flag[robot_id] = False  # update moving state of robot
                    #         print(c + "[Request CancelMove1]\t{RobotID}".format(RobotID=robot_id))
                    #         cancel_response = self.request(self.BI_name[robot_id], Cancel_gl)  # get response of Cancel request
                    #         cancel_response_gl = GLFactory.new_gl_from_gl_string(cancel_response)
                    #         if cancel_response_gl.get_name() == "fail":
                    #             print(c + "[Response CancelMove1]\t{RobotID}: FAIL".format(RobotID=robot_id))
                    #             time.sleep(1)
                    #             continue
                    #         else:
                    #             result = cancel_response_gl.get_expression(1).as_value().string_value()  # "success" if Cancel request is done
                    #             print(c + "[Response CancelMove1]\t{RobotID}: {Result}".format(RobotID=robot_id, Result=result))
                    #             time.sleep(0.5)
                    #             break

                    ### Control Request to move ###
                    ''' Move actionID:
                            "AMR_LIFT1": "\"1\""
                            "AMR_LIFT2": "\"2\""
                            "AMR_TOW1": "\"3\""
                            "AMR_TOW2": "\"4\""
                        move gl format: (move (actionID $actionID) $path) '''

                    self.move_flag[robot_id] = True  # update moving state of robot
                    while self.thread_flag[robot_id]:
                        time.sleep(1)
                        # print("[INFO] {RobotID} is waiting for Path Update".format(RobotID=robot_id))
                        if self.ltm.NC.robotTM[robot_id]:
                            robot_path = copy.copy(self.ltm.NC.robotTM[robot_id])
                            # print("[INFO] {RobotID} got new path".format(RobotID=robot_id))
                            break

                    if self.thread_flag[robot_id]:
                        temp_Move_gl = "(move (actionID {actionID}) {path})"
                        path_gl = self.path_gl_generator(robot_path, robot_id)  # convert path list to path gl
                        Move_gl = temp_Move_gl.format(actionID=self.BI_actionID[robot_id][0], path=path_gl)

                        self.SMM_notify(robot_id)
                        while self.thread_flag[robot_id]:
                            print(c + "[Request Move1]\t\t{RobotID}\t{Path}".format(RobotID=robot_id, Path=str(path_gl)))
                            self.move_flag[robot_id] = True
                            move_response = self.request(self.BI_name[robot_id],Move_gl)  # request move control to robotBI, get response of request
                            print(c + "[response Move1]\t\t{RobotID}\t{response}".format(RobotID=robot_id, response=str(move_response)))
                            move_response_gl = GLFactory.new_gl_from_gl_string(move_response)
                            if move_response_gl.get_name() == "fail":
                                time.sleep(1)
                            else:
                                result = move_response_gl.get_expression(1).as_value().string_value()  # "success" if request is done, "(fail)"" if request can't be handled
                                print(c + "[response Move1]\t\t{RobotID}\t{Path}: {Result}".format(RobotID=robot_id, Path=str(path_gl), Result=str(result)))
                                break

                elif len(self.ltm.NC.robotTM_set[robot_id]) >= 2:  # split path (avoiding path)
                    self.avoid_flag[robot_id] = True  # update avoiding state of robot
                    counterpart_check = {0: 1, 1: 0, 2: 3, 3: 2}
                    robot_index = self.AMR_IDs.index(robot_id)
                    c_robot_id = self.AMR_IDs[counterpart_check[robot_index]]  # get robotID of counterpart robot
                    self.avoid_flag[c_robot_id] = True

                    robotTM_set = copy.copy(self.ltm.NC.robotTM_set)  # get path set
                    robotTM_scond = copy.copy(self.ltm.NC.robotTM_scond)  # get start condition of robot
                    ''' self.ltm.NC.robotTM_scond: start condition of split path
                        e.g. AMR_LIFT1 is avoiding robot and AMR_LIFT2 is counterpart robot
                             self.ltm.NC.robotTM_set["AMR_LIFT1"] == [[1, 2, 3], [4, 5, 6, 7]]
                             self.ltm.NC.robotTM_set["AMR_LIFT2"] == [4, 5, 6, 7, 8, 9]
                             self.ltm.NC.robotTM_scond["AMR_LIFT1"] == [[], [["AMR_LIFT2", [0, 1]]]]
    
                        self.ltm.NC.robotTM_scond["AMR_LIFT1"]: start condition of "AMR_LIFT1"(avoidingRobot)
                        -> no condition of 0th path
                        -> [["AMR_LIFT2", [0, 1]]] condition of 1th path which means start after "AMR_LIFT2" passes 1th element(5) in 0th path([4, 5, 6, 7, 8, 9]) '''

                    ### Cancel current control request if robot is moving ###
                    # if self.move_flag[robot_id]:  # check whether robot is moving
                    #     while True:
                    #         temp_Cancel_gl = "(cancelMove (actionID {actionID}))"
                    #         Cancel_gl = temp_Cancel_gl.format(actionID=self.BI_actionID[robot_id][1])  # Cancel actionID
                    #         self.move_flag[robot_id] = False  # update moving state of robot
                    #         print(c + "[Request CancelMove2]\t{RobotID}".format(RobotID=robot_id))
                    #         cancel_response = self.request(self.BI_name[robot_id], Cancel_gl)  # get response of Cancel request
                    #         cancel_response_gl = GLFactory.new_gl_from_gl_string(cancel_response)
                    #         if cancel_response_gl.get_name() == "fail":
                    #             print(c + "[Response CancelMove2]\t{RobotID}: FAIL".format(RobotID=robot_id))
                    #             time.sleep(1)
                    #             continue
                    #         else:
                    #             result = cancel_response_gl.get_expression(1).as_value().string_value()  # "success" if Cancel request is done
                    #             print(c + "[Response CancelMove2]\t{RobotID}: {Result}".format(RobotID=robot_id, Result=result))
                    #             time.sleep(0.5)
                    #             break

                    ### Control request to move ###
                    for path_idx in range(len(robotTM_set[robot_id])):  # consider path set
                        if robotTM_scond[robot_id][path_idx]:  # check whether current path has any start condition
                            wait_flag = True
                            while wait_flag and self.thread_flag[robot_id]:
                                time.sleep(1)
                                print(c + "wait flag")
                                for cond in robotTM_scond[robot_id][path_idx]:
                                    if self.ltm.NC.PlanExecutedIdx[cond[0]][0] < cond[1][0] or \
                                            self.ltm.NC.PlanExecutedIdx[cond[0]][1] < cond[1][1]:
                                        wait_flag = True
                                    else:
                                        wait_flag = False
                                        # print("[INFO] Start Condition of {RobotID} is Satisfied".format(RobotID=robot_id))
                                        break

                        ''' Move actionID:
                                "AMR_LIFT1": "\"1\""
                                "AMR_LIFT2": "\"2\""
                                "AMR_TOW1": "\"3\""
                                "AMR_TOW2": "\"4\""
                            move gl format: (move (actionID $actionID) $path) '''
                        self.move_flag[robot_id] = True  # update moving state of robot
                        print(c + "while start")
                        while self.thread_flag[robot_id]:
                            time.sleep(1)
                            # print("[INFO] {RobotID} is waiting for Path Update".format(RobotID=robot_id))
                            if self.ltm.NC.robotTM[robot_id]:
                                robot_path = robotTM_set[robot_id][path_idx]  # get current path of path set
                                # print("[INFO] {RobotID} got new path".format(RobotID=robot_id))
                                break
                        print(c + "while finish : " + str(self.thread_flag[robot_id]))

                        if self.thread_flag[robot_id]:
                            temp_Move_gl = "(move (actionID {actionID}) {path})"
                            path_gl = self.path_gl_generator(robot_path, robot_id)  # convert path list to path gl
                            Move_gl = temp_Move_gl.format(actionID=self.BI_actionID[robot_id][0], path=path_gl)
                            self.SMM_notify(robot_id)

                            while self.thread_flag[robot_id]:
                                print(c + "[Request Move2]\t\t{RobotID}\t{Path}".format(RobotID=robot_id, Path=str(path_gl)))
                                self.move_flag[robot_id] = True
                                move_response = self.request(self.BI_name[robot_id], Move_gl)  # request move control to robotBI, get response of request                                    print(c + "[response Move1]\t\t{RobotID}\t{response}".format(RobotID=robot_id, response=str(move_response)))
                                print(c + "[response Move2]\t\t{RobotID}\t{response}".format(RobotID=robot_id, response=str(move_response)))
                                move_response_gl = GLFactory.new_gl_from_gl_string(move_response)
                                if move_response_gl.get_name() == "fail":
                                    time.sleep(1)
                                else:
                                    result = move_response_gl.get_expression(1).as_value().string_value()  # "success" if request is done, "(fail)"" if request can't be handled
                                    print(c + "[Response Move2]\t\t{RobotID}\t{Path}: {Result}".format(RobotID=robot_id, Path=str(path_gl), Result=str(result)))
                                    break
                    self.avoid_flag[robot_id] = False  # update avoiding state of robot
                    self.avoid_flag[c_robot_id] = False # update avoiding state of robot

            if collide and self.thread_flag[robot_id]:
                self.collide_flag[robot_id] = False
                counterpart_check = {0: 1, 1: 0, 2: 3, 3: 2}
                robot_index = self.AMR_IDs.index(robot_id)
                c_robot_id = self.AMR_IDs[counterpart_check[robot_index]]  # get robotID of counterpart robot
                self.collide_flag[c_robot_id] = False
        self.thread_flag[robot_id] = False
        with self.lock[robot_id]:
            self.lock[robot_id].notify()
        print(c + "[Info] Control request finish " + str(robot_id))

    def SMM_notify(self, robot_id):  # notify SMM of control information
        ''' RobotPathPlan gl format: (RobotPathPlan $robot_id $goal (path $v_id1 $v_id2 ….)) '''
        if self.ltm.NC.robotTM[robot_id]:
            temp_SMM_gl = "(RobotPathPlan \"{robot_id}\" {goal} {path})"
            robot_path = copy.copy(self.ltm.NC.robotTM[robot_id])
            path_gl = self.path_gl_generator(robot_path, robot_id)  # convert path list to path gl
            SMM_gl = temp_SMM_gl.format(robot_id=robot_id, goal=self.actual_goal[robot_id], path=path_gl)
            # print("[Notify] Notify Path and Goal of {RobotID} to SMM".format(RobotID=robot_id))
            self.notify(self.SMM_name, SMM_gl)  # notify SMM
        else:
            # print("[INFO] {RobotID} has no path".format(RobotID=robot_id))
            pass

    def path_gl_generator(self, path, robot_id):  # convert path list to path gl
        ''' path gl format: (path $v_id1 $v_id2 ….) '''
        
        if path[0] != self.cur_robot_pose[robot_id][0]:
            path.insert(0, self.cur_robot_pose[robot_id][0])
        path_gl = "(path"
        for path_i in path:
            path_gl += " "
            path_gl += str(path_i)
        path_gl += ")"

        return path_gl

    def Goal_check(self):  # check whether robot terminates its goal and if it terminates, send result of goal to robotTM
        while True:
            time.sleep(1)
            print("GOAL", self.move_flag, self.ltm.NC.Flag_terminate)
            for robot_id in self.AMR_IDs:
                # if robot_id == "AMR_LIFT2":
                # print("GOAL", robot_id, self.move_flag[robot_id], self.ltm.NC.Flag_terminate[robot_id])
                goal_end_index = self.ltm.NC.Flag_terminate[robot_id]  # get terminateFlag from NC (-1: Not terminated, 0: Success Terminate 1: Fail Terminate)
                # if robot_id == "AMR_TOW1":
                # print("GOAL " + "[" + str(robot_id) + "]" + str(self.move_flag[robot_id]) + " " + str(goal_end_index))
                if self.move_flag[robot_id] and (goal_end_index == 0):  # check wheather robot is moving and just terminates its goal
                    self.move_flag[robot_id] = False  # update state of robot to not moving
                    ''' MoveResult gl format: (MoveResult (actionID $actionID) $robotID $result) '''

                    goal_result_gl = "(MoveResult (actionID \"{ActionID}\") \"{Result}\")".format(
                        ActionID=self.goal_actionID[robot_id],
                        Result="success")
                    print("[INFO] \"{RobotID}\" to \"{GoalID}\": success".format(RobotID=robot_id, GoalID=self.actual_goal[robot_id]))
                    self.send(self.TM_name[robot_id], goal_result_gl)  # send result to robotTM
                    self.actual_goal[robot_id] = self.ltm.NC.robotGoal[robot_id]
                    self.real_goal[robot_id] = self.ltm.NC.robotGoal[robot_id]
                else:
                    pass

    def MultiRobotPath_update(self, response_gl):  # update paths of robots in NC
        ''' MultiRobotPath gl format: (MultiRobotPath (RobotPath $robot_id (path $v_id1 $v_id2 $v_id3, ...)), …) '''
        print('multi robot path update : ' + str(response_gl))
        multipaths = dict()  # {robotoID: path}
        robot_num = response_gl.get_expression_size()  # get number of robots
        robot_pose = {}  # {robotID: [vertex, vertex]}
        robot_ids = []
        for i in range(robot_num):
            ''' RobotPath gl format: (RobotPath $robot_id (path $v_id1 $v_id2 $v_id3, ...)) '''
            robot_info = response_gl.get_expression(i).as_generalized_list()  # get RobotPath information from gl
            robot_id = robot_info.get_expression(0).as_value().string_value()  # get robotID
            robot_ids.append(robot_id)
            path_gl = robot_info.get_expression(1).as_generalized_list()  # get path
            path_size = path_gl.get_expression_size()  # get path size
            path_size -= 1
            path = list()
            
            if path_size > 0:
                for j in range(path_size):
                    path.append(path_gl.get_expression(j).as_value().int_value())  # generate path list
            multipaths[robot_id] = path
            robot_pose[robot_id] = copy.copy(self.cur_robot_pose[robot_id])  # get current pose of robot
            print('robot ID : ' + str(robot_id))
            print('path : ' + str(path))

        # print("before multipath plan")
        self.ltm.NC.get_multipath_plan(multipaths)  # update path in NC
        # print("after multipath plan")
        self.ltm.NC.update_robot_TM(robot_pose)  # update pose in NC

        return robot_ids

    def MultiRobotPath_query(self, robot_id_replan):  # query about path to MultiAgentPathFinder(MAPF)
        ''' MultiRobotPath gl format: (MultiRobotPath (RobotPath $robot_id $cur_vertex $goal_id), …) '''

        path_query_gl = "(MultiRobotPath"
        for robot_id in robot_id_replan:
            # start_id = copy.deepcopy(self.cur_robot_pose[robot_id][0])  # get current nearest vertext to set start vertex
            # start_id = copy.copy(self.cur_robot_pose[robot_id][0])  # get current nearest vertext to set start vertex
            start_id = self.cur_robot_pose[robot_id][0]
            # goal_id = copy.deepcopy(self.ltm.NC.robotGoal[robot_id]) # get current goal
            # goal_id = copy.deepcopy(self.real_goal[robot_id])
            goal_id = copy.copy(self.real_goal[robot_id])
            if goal_id == -1:  # check whether robot has no goal
                goal_id = start_id  # no gal -> start==goal
            path_query_gl += " (RobotPath \"" + robot_id + "\" " + str(start_id) + " " + str(goal_id) + ")"

            ''' MultiRobotPath querh should have all of robots in same type
                e.g. TOW1/TOW2 or LIFT1/LIFT2 '''

            if len(robot_id_replan) == 1:  # check wheather robot_id_replan(argument) has one ID
                ''' if only has
                        LIFT1 -> add LIFT2
                        LIFT2 -> add LIFT1
                        TOW1 -> add TOW2
                        TOW2 -> add TOW1 '''

                counterpart_check = {0: 1, 1: 0, 2: 3, 3: 2}
                robot_index = self.AMR_IDs.index(robot_id)
                c_robot_id = self.AMR_IDs[counterpart_check[robot_index]]  # get robotID of counterpart robot
                # c_start_id = copy.deepcopy(self.cur_robot_pose[c_robot_id][0])  # get current nearest vertext
                # c_goal_id = copy.deepcopy(self.real_goal[c_robot_id])  # get current goal
                c_start_id = copy.copy(self.cur_robot_pose[c_robot_id][0])  # get current nearest vertext
                c_goal_id = copy.copy(self.real_goal[c_robot_id])  # get current goal
                if c_goal_id == -1:  # check whether robot has no goal
                    c_goal_id = c_start_id  # no gal -> start==goal
                path_query_gl += " (RobotPath \"" + c_robot_id + "\" " + str(c_start_id) + " " + str(c_goal_id) + ")"

        path_query_gl += ")"

        time.sleep(0.05)
        print("[Query] Query Robot Path to MAPF :", path_query_gl)
        path_response = self.query(agent_MAPF, path_query_gl)
        print("[Query] Response from MAPF :", path_response)

        return path_response


if __name__ == "__main__":
    agent = NavigationControllerAgent()
    arbi_agent_executor.execute(broker_url=broker_url,
                                agent_name="agent://www.arbi.com/Local/NavigationController",
                                agent=agent, broker_type=2)  # same role with agent.initialize
    while True:
        pass
