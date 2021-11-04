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
        
        self.broker = broker_url # broker address
        self.connect(self.broker, "ds://www.arbi.com/Local/NavigationController", 2) # connect broker

        self.map_file = str(pathlib.Path(__file__).parent.parent.resolve()) + "/data/map_cloud.txt" # load map file (~/data/map_cloud.txt)
        self.MAP = MapMOS(self.map_file) # interprete map
        
        self.AMR_IDs = ["AMR_LIFT1", "AMR_LIFT2", "AMR_TOW1", "AMR_TOW2"] # AMR IDs
        self.AMR_LIFT_IDs = list() # LIFT IDs
        self.AMR_TOW_IDs = list() # TOW IDs
        for identifier in self.AMR_IDs:
            if "LIFT" in identifier:
                self.AMR_LIFT_IDs.append(identifier)
            elif "TOW" in identifier:
                self.AMR_TOW_IDs.append(identifier)

        self.NC = NavigationControl(self.AMR_IDs, self.AMR_LIFT_IDs, self.AMR_TOW_IDs) # launch NC(NavigationController)

class NavigationControlerAgent(ArbiAgent):
    def __init__(self):
        super().__init__()
        self.ltm = None # ltm address (to be modified after on_start)
        self.lock = Condition()
        
        self.TM_name = {"AMR_LIFT1": "agent://www.arbi.com/Lift1/TaskManager",
                        "AMR_LIFT2": "agent://www.arbi.com/Lift2/TaskManager",
                        "AMR_TOW1": "agent://www.arbi.com/Tow1/TaskManager",
                        "AMR_TOW2": "agent://www.arbi.com/Tow2/TaskManager"} # agent address of robotTaskManager(robotTM)

        self.BI_name = {
            "AMR_LIFT1": "agent://www.arbi.com/Lift1/BehaviorInterface",
            "AMR_LIFT2": "agent://www.arbi.com/Lift2/BehaviorInterface",
            "AMR_TOW1": "agent://www.arbi.com/Tow1/BehaviorInterface",
            "AMR_TOW2": "agent://www.arbi.com/Tow2/BehaviorInterface"
        }  # agent adress of robotBehaviorInterface(robotBI)
        
        self.SMM_name = "agent://www.arbi.com/Local/MapManager" # agent address of SemanticMapManager(SMM)
        
        self.BI_actionID = {
            "AMR_LIFT1": ["\"1\"", "\"5\""],
            "AMR_LIFT2": ["\"2\"", "\"6\""],
            "AMR_TOW1": ["\"3\"", "\"7\""],
            "AMR_TOW2": ["\"4\"", "\"8\""]
        } # actionID [moveID, cancelID] to send to robotBI

        self.AMR_IDs = ["AMR_LIFT1", "AMR_LIFT2", "AMR_TOW1", "AMR_TOW2"] # AMR IDs
        self.AMR_LIFT_IDs = list() # LIFT IDs
        self.AMR_TOW_IDs = list() # TOW IDs
        for identifier in self.AMR_IDs:
            if "LIFT" in identifier:
                self.AMR_LIFT_IDs.append(identifier)
            elif "TOW" in identifier:
                self.AMR_TOW_IDs.append(identifier)

        self.cur_robot_pose = {} # current pose of robot (notify from SMM) e.g. {robotID: [vertex, vertex]}
        self.goal_actionID = {} # actionID from TM(goal request) e.g. {robotID: actionID}
        self.robot_goal = {} # current goal of robot (request from TM) e.g. {robotID: goalVertex}
        self.move_flag = {
            "AMR_LIFT1": False,
            "AMR_LIFT2": False,
            "AMR_TOW1": False,
            "AMR_TOW2": False
        } # identify whether robot is moving
        self.avoid_flag = {
            "AMR_LIFT": False,
            "AMR_LIFT2": False,
            "AMR_TOW1": False,
            "AMR_TOW2": False
        } # identify whether robot is in avoiding plan
        self.collide_flag = {
            "AMR_LIFT": False,
            "AMR_LIFT2": False,
            "AMR_TOW1": False,
            "AMR_TOW2": False
        } # identify whether robot is in avoiding plan
        
    def on_start(self): # executed when the agent initializes
        self.ltm = NavigationControlerDataSource(broker_url) # ltm class
        self.ltm.connect(broker_url, "ds://www.arbi.com/Local/NavigationController", 2) # connect ltm

        time.sleep(1)

        Thread(target=self.Goal_check, args=(), daemon=True).start() # chehck goal status of each robot every 1sec

    def on_data(self, sender: str, data: str): # N/A
        print(data)

    def on_notify(self, sender, notification): # executed when the agent gets notification
        print("[on Notify] " + notification)
        temp_gl = GLFactory.new_gl_from_gl_string(notification) # notification to gl
        gl_name = temp_gl.get_name() # get name of gl
        result = (gl_name == "MultiRobotPose") # identifier
        if result: # "MultiRobotPose" update from SMM
            ''' MultiRobotPose gl format: (MultiRobotPose (RobotPose $robot_id (vertex $vertex_id $vertex_id)), ...) '''

            robot_num = temp_gl.get_expression_size() # check updated number of robots
            multi_robot_pose = {} # {robotID: [vertex, vertex]}

            for i in range(robot_num): # update robot pose from gl
                '''RobotPose_gl format: (RobotPose $robot_id (vertex $vertex_id $vertex_id))'''

                RobotPose_gl = temp_gl.get_expression(i).as_generalized_list() # get ith gl content
                robot_id = RobotPose_gl.get_expression(0).as_value().string_value() # get robotID
                robot_vertex_gl = RobotPose_gl.get_expression(1).as_generalized_list() # get current pose (vertex)
                robot_vertex = [robot_vertex_gl.get_expression(0).as_value().int_value(),
                                robot_vertex_gl.get_expression(1).as_value().int_value()] # current pose to list
                multi_robot_pose[robot_id] = robot_vertex
                self.cur_robot_pose[robot_id] = robot_vertex # update current robot pose in agent

            robot_sendTM = self.ltm.NC.update_robot_TM(multi_robot_pose) # update robot pose in NC
            ''' robot_sendTM: robotID that has any change of its path and notify the information of robotID to robotBI '''
            
            if robot_sendTM: # check whether robot_sendTM is empty
                for robot_id in robot_sendTM:
                    self.SMM_notify(robot_id) # # notify SMM of same control information
                    # if not (self.avoid_flag[robot_id] or self.move_flag[robot_id]): # check whether robot is in avoiding plan
                    if not self.avoid_flag[robot_id]:
                        print("Control request by <POSE> Notification")
                        Thread(target=self.Control_request, args=(robot_id, ), daemon=True).start() # control request of robotID
                   
        elif gl_name == "Collidable": # "Collidable" update from SMM
            ''' Collidable gl format: (Collidable $num (pair $robot_id $robot_id $time), …) 
                                    num: number of collidable set
                                    time: in which collision is expected '''

            collide_num = temp_gl.get_expression(0).as_value().int_value() # get number of collidable set
            for i in range(collide_num):
                ''' collidable_set(pair) gl format: (pair $robot_id $robot_id $time) '''
                
                collidable_set_gl = temp_gl.get_expression(i+1).as_generalized_list() # get ith collidable set
                robot_ids = [collidable_set_gl.get_expression(0).as_value().string_value(), collidable_set_gl.get_expression(1).as_value().string_value()] # get robotIDs of collidable set
                check = 0
                for robot_id in robot_ids:
                    check += self.collide_flag[robot_id]
                if check==0:
                    self.ltm.NC.update_start_goal_collision(robot_ids) # update collidable robotIDs in NC
                    
                    response = self.MultiRobotPath_query(robot_ids) # query about not collidable path to MultiAgentPathFinder(MAPF)
                    time.sleep(0.5)
                    response_gl = GLFactory.new_gl_from_gl_string(response)
                    ''' response gl format: (MultiRobotPath (RobotPath $robot_id (path $v_id1 $v_id2 $v_id3, ...)), …) '''

                    self.MultiRobotPath_update(response_gl) # update path of robot in NC
                    for robot_id in robot_ids:
                        self.SMM_notify(robot_id) # # notify SMM of same control information
                        print("Control request by <COLLIDABLE> Notification")
                        Thread(target=self.Control_request, args=(robot_id, True, False, ), daemon=True).start()

    def on_request(self, sender, request): # executed when the agent gets request
        print("[on Request] " + request)
        temp_gl = GLFactory.new_gl_from_gl_string(request)

        if temp_gl.get_name() == "Move": # "Move" request from robotTM
            ''' Move gl format: (Move (actionID $actionID) $robotID $start $end)
                                start: start vertex
                                end: end(goal) vertex '''

            robot_goal = {} # {robotID: goalVertex}
            action_id = temp_gl.get_expression(0).as_generalized_list().get_expression(0).as_value().string_value() # get actionID
            robot_id = temp_gl.get_expression(1).as_value().string_value() # get robotID
            self.goal_actionID[robot_id] = action_id # update actionID in agent
            temp_start = temp_gl.get_expression(2).as_value().int_value()
            goal_vertex = temp_gl.get_expression(3).as_value().int_value() # get goalVertex
            robot_goal[robot_id] = goal_vertex # robotID-goalVertex pair
            self.robot_goal[robot_id] = goal_vertex # update goalVertex in agent
            robot_id_replan, robot_id_BI = self.ltm.NC.allocate_goal(robot_goal, self.cur_robot_pose) # allocate goal in NC
            ''' robot_id_replan: robot which needs to get replanning -> MAPF
                robot_id_BI: robotID that has any change of its path and notify the information of robotID to robotBI '''
            
            print("TM: ", robot_id_BI)
            print("replan: ", robot_id_replan)
            if robot_id_BI: # check whether robot_id_BI is empty
                for robot_id in robot_id_BI:
                    self.SMM_notify(robot_id) # # notify SMM of same control information
                    print("Control request by <Move> Request")
                    Thread(target=self.Control_request, args=(robot_id, ), daemon=True).start() # control request of robotID

            if robot_id_replan: # check whether robot_id_replan is empty
                response = self.MultiRobotPath_query(robot_id_replan) # query about not collidable path to MultiAgentPathFinder(MAPF)
                response_gl = GLFactory.new_gl_from_gl_string(response)
                ''' response gl format: (MultiRobotPath (RobotPath $robot_id (path $v_id1 $v_id2 $v_id3, ...)), …) '''

                robot_ids = self.MultiRobotPath_update(response_gl) # update path of robot in NC
                for robot_id in robot_ids:
                    self.SMM_notify(robot_id) # # notify SMM of same control information
                    print("Control request by <Move> Request")
                    Thread(target=self.Control_request, args=(robot_id, False, True, ), daemon=True).start() # control request of robotID

            goal_request_result = "(ok)" # response to robotTM that request is successful

            return goal_request_result
        else:
            print("what?", str(temp_gl))
            return "(fail)"

    def Control_request(self, robot_id, collide=False, replan=False):
        ''' if robot is in stationary state, output from MAPF path is same with current vertex
            e.g. "AMR_TOW1" : in stationary state, no plan to move
                -> self.cur_robot_pose["AMR_TOW1"][0] == self.cur_robot_pose["AMR_TOW1"][1]
                -> len(self.ltm.NC.robotTM[robot_id][0]) == 1
            => no control is needed '''

        ''' self.ltm.NC.robotTM[robotID]: current path of robotID '''
        
        
        if collide:
            self.collide_flag[robot_id] = True
            counterpart_check = {0: 1, 1: 0, 2: 3, 3: 2}
            robot_index = self.AMR_IDs.index(robot_id)
            c_robot_id = self.AMR_IDs[counterpart_check[robot_index]] # get robotID of counterpart robot
            self.collide_flag[c_robot_id] = True
            time.sleep(1)
        
        elif replan:
            time.sleep(1)

        if (len(self.ltm.NC.robotTM[robot_id]) == 1): # check whether robot is stationary
            stationary_check = (self.cur_robot_pose[robot_id][0] == self.cur_robot_pose[robot_id][1] == self.ltm.NC.robotTM[robot_id][0]) # True if robot is stationary and will be stationary
        else: 
            stationary_check = False
        
        if self.ltm.NC.robotTM[robot_id] and (not stationary_check): # check whether robot has path and not stationary
            ''' if robot is avoiding against counterpart robot, path of the robot is split
                e.g. self.ltm.NC.robotTM_set[avoidingRobotID] == [[path], [path]]
                     self.ltm.NC.robotTM_set[counterpartRobotID] == [[path]] '''

            if len(self.ltm.NC.robotTM_set[robot_id]) == 1: # not split path (not avoiding path)
                ### Cancel current control request if robot is moving ###
                if self.move_flag[robot_id]: # check whether robot is moving
                    ''' If robot is moving now, current path should be canceled.
                        Cancel actionID:
                            "AMR_LIFT1": "\"5\""
                            "AMR_LIFT2": "\"6\""
                            "AMR_TOW1": "\"7\""
                            "AMR_TOW2": "\"8\""
                        cancelMove gl format: (cancelMove (actionID $actionID)) '''

                    temp_Cancel_gl = "(cancelMove (actionID {actionID}))"
                    Cancel_gl = temp_Cancel_gl.format(actionID=self.BI_actionID[robot_id][1]) # Cancel actionID
                    self.move_flag[robot_id] = False # update moving state of robot
                    response = self.request(self.BI_name[robot_id], Cancel_gl) # get response of Cancel request
                    response_gl = GLFactory.new_gl_from_gl_string(response)
                    result = response_gl.get_expression(1).as_value().string_value() # "success" if Cancel request is done
                        
                    print("Cancel {robot_id} path : {result}".format(
                        robot_id=robot_id,
                        result=str(result)))
                    time.sleep(0.5)


                ### Control Request to move ###
                ''' Move actionID:
                        "AMR_LIFT1": "\"1\""
                        "AMR_LIFT2": "\"2\""
                        "AMR_TOW1": "\"3\""
                        "AMR_TOW2": "\"4\""
                    move gl format: (move (actionID $actionID) $path) '''
                self.move_flag[robot_id] = True # update moving state of robot
                while True:
                        time.sleep(1)
                        print(robot_id, ": Waiting for path update")
                        if self.ltm.NC.robotTM[robot_id]:
                            robot_path = copy.deepcopy(self.ltm.NC.robotTM[robot_id]) # get current path of path set
                            print("got new path")
                            break

                temp_Move_gl = "(move (actionID {actionID}) {path})"
                path_gl = self.path_gl_generator(robot_path, robot_id) # convert path list to path gl
                Move_gl = temp_Move_gl.format(
                    actionID=self.BI_actionID[robot_id][0], 
                    path=path_gl)
                
                print("Request {robot_id} to go {next_vertex}".format(
                    robot_id=robot_id,
                    next_vertex=path_gl))

                # self.SMM_notify(robot_id) # # notify SMM of same control information
                # self.move_flag[robot_id] = True # update moving state of robot
                response = self.request(self.BI_name[robot_id], Move_gl) # request move control to robotBI, get response of request
                response_gl = GLFactory.new_gl_from_gl_string(response)
                result = response_gl.get_expression(1).as_value().string_value() # "success" if request is done, "(fail)"" if request can't be handled

                print("Request {robot_id} to go {next_vertex} : {result}".format(
                    robot_id=robot_id,
                    next_vertex=path_gl,
                    result=result))

            elif len(self.ltm.NC.robotTM_set[robot_id]) >= 2: # split path (avoiding path)
                self.avoid_flag[robot_id] = True # update avoiding state of robot
                counterpart_check = {0: 1, 1: 0, 2: 3, 3: 2}
                robot_index = self.AMR_IDs.index(robot_id)
                c_robot_id = self.AMR_IDs[counterpart_check[robot_index]] # get robotID of counterpart robot
                self.avoid_flag[c_robot_id] = True

                robotTM_set = copy.deepcopy(self.ltm.NC.robotTM_set) # get path set
                robotTM_scond = copy.deepcopy(self.ltm.NC.robotTM_scond) # get start condition of robot
                ''' self.ltm.NC.robotTM_scond: start condition of split path
                    e.g. AMR_LIFT1 is avoiding robot and AMR_LIFT2 is counterpart robot
                         self.ltm.NC.robotTM_set["AMR_LIFT1"] == [[1, 2, 3], [4, 5, 6, 7]]
                         self.ltm.NC.robotTM_set["AMR_LIFT2"] == [4, 5, 6, 7, 8, 9]
                         self.ltm.NC.robotTM_scond["AMR_LIFT1"] == [[], [["AMR_LIFT2", [0, 1]]]]
                         
                    self.ltm.NC.robotTM_scond["AMR_LIFT1"]: start condition of "AMR_LIFT1"(avoidingRobot)
                    -> no condition of 0th path
                    -> [["AMR_LIFT2", [0, 1]]] condition of 1th path which means start after "AMR_LIFT2" passes 1th element(5) in 0th path([4, 5, 6, 7, 8, 9]) '''
                
                ### Cancel current control request if robot is moving ###
                if self.move_flag[robot_id]: # check whether robot is moving
                    ''' If robot is moving now, current path should be canceled.
                        Cancel actionID:
                            "AMR_LIFT1": "\"5\""
                            "AMR_LIFT2": "\"6\""
                            "AMR_TOW1": "\"7\""
                            "AMR_TOW2": "\"8\""
                        cancelMove gl format: (cancelMove (actionID $actionID)) '''

                    temp_Cancel_gl = "(cancelMove (actionID {actionID}))"
                    Cancel_gl = temp_Cancel_gl.format(actionID=self.BI_actionID[robot_id][1]) # Cancel actionID
                    self.move_flag[robot_id] = False # update moving state of robot
                    response = self.request(self.BI_name[robot_id], Cancel_gl) # get response of Cancel request
                    print(response)
                    response_gl = GLFactory.new_gl_from_gl_string(response)
                    result = response_gl.get_expression(1).as_value().string_value() # "success" if Cancel request is done
                        
                    print("Cancel {robot_id} path : {result}".format(
                        robot_id=robot_id,
                        result=str(result)))
                    time.sleep(0.5)
                

                ### Control request to move ###
                for path_idx in range(len(robotTM_set[robot_id])): # consider path set
                    if robotTM_scond[robot_id][path_idx]: # check whether current path has any start condition
                        print("scond ", robotTM_scond)
                        wait_flag = True
                        while wait_flag:
                            time.sleep(1)
                            for cond in robotTM_scond[robot_id][path_idx]:
                                if self.ltm.NC.PlanExecutedIdx[cond[0]][0]<cond[1][0] or self.ltm.NC.PlanExecutedIdx[cond[0]][1]<cond[1][1]:
                                    wait_flag = True
                                else:
                                    wait_flag = False
                                    print(robot_id, "Next Path Segment")
                                    break

                        # while True: # if it has start condition, wait until it satisfies that condition
                        #     counterpart_id = robotTM_scond[robot_id][path_idx][0][0] # get robotID of counterpartRobot
                        #     counterpart_idx = robotTM_scond[robot_id][path_idx][0][1] # get condition of counterpartRobbot
                        #     counterpart_pose = self.cur_robot_pose[counterpart_id] # get current pose of counterpartRobbot
                        #     counterpart_set = robotTM_set[counterpart_id] # get path set of counterpartRobbot
                        #     # counterpart_check = (counterpart_pose[0] == counterpart_set[counterpart_idx[0]][counterpart_idx[1]]) and (
                        #     #     counterpart_pose[1] == counterpart_set[counterpart_idx[0]][counterpart_idx[1]]) # check whether it satisfies the condition
                        #     counterpart_check = (counterpart_set[counterpart_idx[0]][counterpart_idx[1]] in counterpart_pose)
                        #     if counterpart_check: # break if it satisfies
                        #         print(robot_id, "Next Path Segment")
                        #         time.sleep(2)
                        #         break
                        #     else:
                        #         time.sleep(1)
                            

                    ''' Move actionID:
                            "AMR_LIFT1": "\"1\""
                            "AMR_LIFT2": "\"2\""
                            "AMR_TOW1": "\"3\""
                            "AMR_TOW2": "\"4\""
                        move gl format: (move (actionID $actionID) $path) '''
                    self.move_flag[robot_id] = True # update moving state of robot
                    while True:
                        time.sleep(1)
                        print(robot_id, ": Waiting for update")
                        if self.ltm.NC.robotTM[robot_id]:
                            robot_path = robotTM_set[robot_id][path_idx] # get current path of path set
                            print("got new path")
                            break

                    # robot_path = robotTM_set[robot_id][path_idx] # get current path of path set
                    # if self.cur_robot_pose[robot_id][0]==self.cur_robot_pose[robot_id][1]:
                    #     if self.cur_robot_pose[robot_id][0] != robot_path[0]:
                    #         robot_path.insert(0, self.cur_robot_pose[robot_id][0])
                    temp_Move_gl = "(move (actionID {actionID}) {path})"
                    path_gl = self.path_gl_generator(robot_path, robot_id) # convert path list to path gl

                    Move_gl = temp_Move_gl.format(
                        actionID=self.BI_actionID[robot_id][0],
                        path=path_gl)

                    print("Request {robot_id} to go {next_vertex}".format(
                        robot_id=robot_id,
                        next_vertex=path_gl))

                    # self.SMM_notify(robot_id) # notify SMM of same control information
                    # self.move_flag[robot_id] = True # update moving state of robot
                    response = self.request(self.BI_name[robot_id], Move_gl) # request move control to robotBI, get response of request
                    response_gl = GLFactory.new_gl_from_gl_string(response)
                    result = response_gl.get_expression(1).as_value().string_value() # "success" if request is done, "(fail)"" if request can't be handled

                    print("Request {robot_id} to go {next_vertex} : {result}".format(
                        robot_id=robot_id,
                        next_vertex=path_gl,
                        result=result))

                self.avoid_flag[robot_id] = False # update avoiding state of robot
            
        if collide:
            self.collide_flag[robot_id] = False
            counterpart_check = {0: 1, 1: 0, 2: 3, 3: 2}
            robot_index = self.AMR_IDs.index(robot_id)
            c_robot_id = self.AMR_IDs[counterpart_check[robot_index]] # get robotID of counterpart robot
            self.collide_flag[c_robot_id] = False
            
        # else:
        #     print(robot_id, " no path")
        #     self.SMM_notify(robot_id)

    def SMM_notify(self, robot_id): # notify SMM of control information
        ''' RobotPathPlan gl format: (RobotPathPlan $robot_id $goal (path $v_id1 $v_id2 ….)) '''

        temp_SMM_gl = "(RobotPathPlan \"{robot_id}\" {goal} {path})"
        robot_path = copy.deepcopy(self.ltm.NC.robotTM[robot_id])
        path_gl = self.path_gl_generator(robot_path, robot_id) # convert path list to path gl
        SMM_gl = temp_SMM_gl.format(
            robot_id=robot_id,
            goal=self.robot_goal[robot_id],
            path=path_gl)
        self.notify(self.SMM_name, SMM_gl) # notify SMM

    def path_gl_generator(self, path, robot_id): # convert path list to path gl
        ''' path gl format: (path $v_id1 $v_id2 ….) '''
        
        if path[0]!=self.cur_robot_pose[robot_id][0]:
            path.insert(0, self.cur_robot_pose[robot_id][0])
        
        path_gl = "(path"
        for path_i in path:
            path_gl += " "
            path_gl += str(path_i)
        path_gl += ")"

        return path_gl

    def Goal_check(self): # check whether robot terminates its goal and if it terminates, send result of goal to robotTM
        while True:
            time.sleep(1)
            print("AMR_TOW2: ", self.ltm.NC.robotTM["AMR_TOW2"])
            for robot_id in self.AMR_IDs:
                goal_end_index = self.ltm.NC.Flag_terminate[robot_id] # get terminateFlag from NC (-1: Not terminated, 0: Success Terminate 1: Fail Terminate)
                # print(robot_id, self.move_flag, goal_end_index)
                if self.move_flag[robot_id] and (goal_end_index == 0): # check wheather robot is moving and just terminates its goal
                    self.move_flag[robot_id] = False # update state of robot to not moving
                    ''' MoveResult gl format: (MoveResult (actionID $actionID) $robotID $result) '''

                    goal_result_gl = "(MoveResult (actionID \"{actionID}\") \"{robotID}\" \"{result}\")".format(
                        actionID=self.goal_actionID[robot_id],
                        robotID=robot_id,
                        result="success")
                    print("[INFO] \"{robotID}\" to \"{goalID}\": success".format(robotID=robot_id, goalID=self.robot_goal[robot_id]))
                    self.send(self.TM_name[robot_id], goal_result_gl) # send result to robotTM
                else:
                    pass

    def MultiRobotPath_update(self, response_gl): # update paths of robots in NC
        ''' MultiRobotPath gl format: (MultiRobotPath (RobotPath $robot_id (path $v_id1 $v_id2 $v_id3, ...)), …) '''

        multipaths = dict() # {robotoID: path}
        robot_num = response_gl.get_expression_size() # get number of robots
        robot_pose = {} # {robotID: [vertex, vertex]}
        robot_ids = []
        for i in range(robot_num):
            ''' RobotPath gl format: (RobotPath $robot_id (path $v_id1 $v_id2 $v_id3, ...)) '''

            robot_info = response_gl.get_expression(i).as_generalized_list() # get RobotPath information from gl
            robot_id = robot_info.get_expression(0).as_value().string_value() # get robotID
            robot_ids.append(robot_id)
            path_gl = robot_info.get_expression(1).as_generalized_list() # get path
            path_size = path_gl.get_expression_size() # get path size
            path = list()
            for j in range(path_size):
                path.append(path_gl.get_expression(j).as_value().int_value()) # generate path list
            multipaths[robot_id] = path
            robot_pose[robot_id] = copy.deepcopy(self.cur_robot_pose[robot_id]) # get current pose of robot
        
        self.ltm.NC.get_multipath_plan(multipaths) # update path in NC
        self.ltm.NC.update_robot_TM(robot_pose) # update pose in NC

        return robot_ids
            
    def MultiRobotPath_query(self, robot_id_replan): # query about path to MultiAgentPathFinder(MAPF)
        ''' MultiRobotPath gl format: (MultiRobotPath (RobotPath $robot_id $cur_vertex $goal_id), …) '''
    
        request_gl = "(MultiRobotPath"
        for robot_id in robot_id_replan:
            start_id = copy.deepcopy(self.cur_robot_pose[robot_id][0]) # get current nearest vertext to set start vertex
            goal_id = copy.deepcopy(self.ltm.NC.robotGoal[robot_id]) # get current goal
            if goal_id == -1: # check whether robot has no goal
                goal_id = start_id # no gal -> start==goal
            request_gl += " (RobotPath \"" + robot_id + "\" " + str(start_id) + " " + str(goal_id) + ")"

            ''' MultiRobotPath querh should have all of robots in same type
                e.g. TOW1/TOW2 or LIFT1/LIFT2 '''

            if len(robot_id_replan) == 1: # check wheather robot_id_replan(argument) has one ID
                ''' if only has
                        LIFT1 -> add LIFT2
                        LIFT2 -> add LIFT1
                        TOW1 -> add TOW2
                        TOW2 -> add TOW1 '''

                counterpart_check = {0: 1, 1: 0, 2: 3, 3: 2}
                robot_index = self.AMR_IDs.index(robot_id)
                c_robot_id = self.AMR_IDs[counterpart_check[robot_index]] # get robotID of counterpart robot
                c_start_id = copy.deepcopy(self.cur_robot_pose[c_robot_id][0]) # get current nearest vertext
                c_goal_id = copy.deepcopy(self.ltm.NC.robotGoal[c_robot_id]) # get current goal
                if c_goal_id == -1: # check whether robot has no goal
                    c_goal_id = c_start_id # no gal -> start==goal
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
