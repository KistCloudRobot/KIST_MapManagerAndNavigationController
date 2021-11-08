import copy


class NavigationControl:
    def __init__(self, AMR_IDs, AMR_LIFT_IDs, AMR_TOW_IDs):
        # initialize
        self.AMR_IDs = AMR_IDs
        self.AMR_TOW_IDs = AMR_TOW_IDs
        self.AMR_LIFT_IDs = AMR_LIFT_IDs

        # path given by multi-robot path finder
        # self.NavPath = {}
        # self.timing = {} # execution timing
        # for id in self.AMR_IDs:
        #    self.NavPath[id] = []
        #    self.timing[id] = [] # if NavPath[rid1][idx1] == NavPath[rid2][idx2] and idx2<idx1 : timing[id]=[[rid2, time_idx2, NavPath[rid1][idx1]], ...]

        # command for RobotTM
        self.robotTM = {}  # the current command for RobotTM
        self.robotTM_set = {}  # full sequence of commands for RobotTM
        self.robotTM_scond = {}  # start condition of robotTM
        self.robotGoal = {}  # New: The goal of each robot
        self.robotStart = {}  # New: The goal of each robot
        self.robotPose = {}  # current pose of robot

        for id in self.AMR_IDs:
            self.robotTM[id] = []  # a sequence of vertices
            self.robotTM_set[id] = []  # [robotTM, robotTM, robotTM, ...]
            self.robotTM_scond[id] = []  # start condition of robotTM
            self.robotGoal[id] = -1

        self.PlanExecutedIdx = {}
        self.Flag_terminate = {}  # -1: Not terminated, 0: Success Terminate 1: Fail Terminate
        for id in self.AMR_IDs:
            self.PlanExecutedIdx[id] = [-1, -1]  # [i,j] Save the last index of NavPath[i][j] which the robot follows
            self.Flag_terminate[id] = 0

    # New
    def allocate_goal(self, goals, robot_pose):  # execute when the robot TM allocates a goal to each robot
        # goals: {robot_id: goal vertex, ....}, robot_pose = {id, [vertex, vertex]}

        Rid_robotTM = []  # the list of robot ids that has an updated robotTM
        Rid_replan = []  # the list of robot ids for replanning
        flag_tow = False
        flag_lift = False
        # check if robot_ids are lifts or tows
        for rid in goals.keys():
            if rid in self.AMR_TOW_IDs:
                flag_tow = True
            if rid in self.AMR_LIFT_IDs:
                flag_lift = True

        check_ids = []

        if flag_tow: check_ids.extend(self.AMR_TOW_IDs)
        if flag_lift: check_ids.extend(self.AMR_LIFT_IDs)
        for rid in check_ids:
            '''
            if self.robotGoal[rid] != -1: # the robot has a navigation job => initialize
                print("here5-------------------------")
                Rid_replan.append(rid) # require replanning
                if rid in goals.keys(): # got new job
                    print("here6-------------------------")
                    self.robotGoal[rid] = goals[rid]

                if self.robotTM[rid] != []: # the robot is executing the plan # TODO: test
                    print("here7-------------------------")
                    self.robotTM[rid] = [self.robotTM[rid][0]]
                    Rid_robotTM.append(rid)
                    self.robotStart[rid] = self.robotTM[rid][0]
                    print(str(self.robotStart[rid]))
                    print(str(self.robotTM[rid]))
                    print("here8-------------------------")
                    self.robotTM_set[rid] = [self.robotTM[rid]]  # [robotTM, robotTM, robotTM, ...]
                    self.PlanExecutedIdx[rid] = [0, -1]
                    self.robotTM_scond[rid] = [[]]  # start condition of robotTM

                else:
                    self.robotStart[rid] = robot_pose[rid][0] # start point: the current vertex
                    self.robotTM[rid] = []
                    self.robotTM_set[rid] = []
                    self.PlanExecutedIdx = [-1, -1]
                    self.robotTM_scond = []

            else: # the robot does not have any job
                '''
            if rid in goals.keys():  # get a new job
                Rid_replan.append(rid)
                self.robotStart[rid] = robot_pose[rid][0]
                self.robotGoal[rid] = goals[rid]

        return Rid_replan, Rid_robotTM

    def get_multipath_plan(self, multipaths):  # multipath: MultiPath type
        # initialize PlanExecutedIdx
        print("multipath: ", multipaths)
        for id in multipaths.keys():
            if len(multipaths[id]) == 1:
                stationary_check = (self.robotPose[id][0] == self.robotPose[id][1] == multipaths[id][0])
            else:
                stationary_check = False

            if not stationary_check:
                self.robotTM[id] = []  # a sequence of vertices
                self.robotTM_set[id] = []  # [robotTM, robotTM, robotTM, ...]
                self.robotTM_scond[id] = []  # start condition of robotTM
                self.PlanExecutedIdx[id] = [-1,
                                            -1]  # [i,j] Save the last index of NavPath[i][j] which the robot follows
                self.Flag_terminate[id] = -1

        start_condition = {}
        for key in multipaths.keys():
            start_condition[key] = []
        # Analyze timing
        for rid, path in multipaths.items():

            for idx in range(0, len(path)):  # compare
                scond = []
                rid_list = list(multipaths.keys())
                rid_list.remove(rid)
                for rid2 in rid_list:
                    for idx2 in range(min(idx, len(multipaths[rid2]) - 1), -1, -1):
                        if path[idx] == multipaths[rid2][idx2]:
                            scond.append([rid2, idx2])

                start_condition[rid].append(scond)

        # --------------------------------------
        # modify stat_condition -10 26
        last_idx_init = {}
        start_condition2 = {}
        for rid in multipaths.keys():
            last_idx_init[rid] = -1
            start_condition2[rid] = []

        for rid1 in multipaths.keys():
            last_idx = last_idx_init.copy()
            robot_start_cond = start_condition[rid1].copy()
            for tidx in range(0, len(robot_start_cond)):  # time
                scond = []  # initialize
                for cond in robot_start_cond[tidx]:
                    if cond != []:
                        if last_idx[cond[0]] < cond[1]:
                            last_idx[cond[0]] = cond[1]
                            scond.append(cond)
                    else:
                        scond.append([])

                start_condition2[rid1].append(scond)

        start_condition = start_condition2
        # --------------------------------------

        # Split navigation paths to robotTM
        robotTM_mapping_set = {}
        scondTM_set = {}
        for rid, path in multipaths.items():
            robotTM_seq = []
            robotTM_mapping = []  # save index of robotTM_set corresponding to each element in path
            scond = []
            t1 = 0
            t2 = 0

            if path != []:
                robotTM = [path[0]]
                scond = [start_condition[rid][0]]
                for ii in range(1, len(path)):
                    if start_condition[rid][ii] == []:
                        if path[ii] != robotTM[-1]:
                            robotTM.append(path[ii])
                            t2 = t2 + 1

                    else:
                        robotTM_seq.append(robotTM)
                        t1 = t1 + 1
                        t2 = 0
                        robotTM = [path[ii]]
                        scond.append(start_condition[rid][ii])

                    robotTM_mapping.append([t1, t2])

                robotTM_seq.append(robotTM)

            self.robotTM_set[rid] = copy.copy(robotTM_seq)
            robotTM_mapping_set[rid] = robotTM_mapping
            scondTM_set[rid] = scond

        # find start condnition for TM
        scond_TM_translated = {}
        for rid in multipaths.keys():
            scond = []
            for tt in range(0, len(scondTM_set[rid])):
                if scondTM_set[rid][tt] != []:
                    cond_cur = []
                    for cond in scondTM_set[rid][tt]:
                        cond_cur.append([cond[0], robotTM_mapping_set[cond[0]][cond[1]]])

                    scond.append(cond_cur)
                else:
                    scond.append([])

            #            scond_TM_translated[rid] = scond
            self.robotTM_scond[rid] = scond
        print("TMSET", self.robotTM_set)
        print("EXECUTE", self.PlanExecutedIdx)

    #        self.robotTM_scond = scond_TM_translated

    def update_robot_TM(self, robot_pose):
        Rid_sendRobotTM = []

        # robotTM = copy.deepcopy(self.robotTM)
        # robotTM_set = copy.deepcopy(self.robotTM_set)
        # robotTM_scond = copy.deepcopy(self.robotTM_scond)
        robotTM = copy.copy(self.robotTM)
        robotTM_set = copy.copy(self.robotTM_set)
        robotTM_scond = copy.copy(self.robotTM_scond)
        test = 0
        print("==================================================================================================")
        for rid, vid in robot_pose.items():
            print(rid, "path", robotTM[rid], robotTM_set[rid])
            print(rid, "scond", robotTM_scond[rid])

            self.robotPose[rid] = vid
            robotTM_check = {"current": False, "skip": False}

            if self.robotGoal[rid] != -1:
                if robotTM[rid] != []:
                    if (vid[0] == robotTM[rid][0]) and (vid[1] == robotTM[rid][0]):
                        self.PlanExecutedIdx[rid][1] += 1
                        robotTM_check["current"] = True
                    elif (vid[0] == vid[1]):
                        if vid in robotTM[rid]:
                            vidx = robotTM[rid].index(vid[0])
                            self.PlanExecutedIdx[rid][1] += (vidx + 1)
                            robotTM_check["skip"] = True
                elif (vid[0] == self.robotGoal[rid]) and (vid[1] == self.robotGoal[rid]):
                    self.Flag_terminate[rid] = 0
                    self.robotGoal[rid] = -1

                if robotTM_set[rid] != []:
                    if self.PlanExecutedIdx[rid] == [len(robotTM_set[rid]) - 1, len(robotTM_set[rid][-1]) - 1]:
                        if (vid[0] == self.robotGoal[rid]) and (vid[1] == self.robotGoal[rid]):
                            self.Flag_terminate[rid] = 0
                            self.robotGoal[rid] = -1
                else:
                    if (vid[0] == self.robotGoal[rid]) and (vid[1] == self.robotGoal[rid]):
                        self.Flag_terminate[rid] = 0
                        self.robotGoal[rid] = -1

            if self.Flag_terminate[rid] == -1:
                if robotTM_set[rid] != []:
                    # if (robotTM[rid]!=[]) and (self.PlanExecutedIdx[rid][1]!=-1):
                    if robotTM[rid] != []:
                        # if (vid[0]=robotTM[rid][0]) and (vid[1]==robotTM[rid][1]):
                        if robotTM_check["current"]:
                            temp_path = robotTM[rid]
                            # print("delete", rid, temp_path[0])
                            # print("before delete", rid, temp_path)
                            del temp_path[0]
                            # self.robotTM[rid] = copy.deepcopy(temp_path)
                            self.robotTM[rid] = copy.copy(temp_path)
                            # print("after delete", rid, self.robotTM[rid])
                            # print(rid, self.PlanExecutedIdx)
                        # elif (vid[0]==vid[1]):
                        elif robotTM_check["skip"]:
                            vidx = robotTM[rid].index(vid[0])
                            del robotTM[rid][:vidx + 1]
                            # self.robotTM[rid] = copy.deepcopy(robotTM[rid])
                            self.robotTM[rid] = copy.copy(robotTM[rid])
                    else:
                        start_idx = self.PlanExecutedIdx[rid][0] + 1
                        if robotTM_scond[rid][start_idx] == []:
                            # robotTM[rid] = copy.deepcopy(robotTM_set[rid][start_idx])
                            # self.robotTM[rid] = copy.deepcopy(robotTM[rid])
                            robotTM[rid] = copy.copy(robotTM_set[rid][start_idx])
                            self.robotTM[rid] = copy.copy(robotTM[rid])
                            self.PlanExecutedIdx[rid] = [start_idx, -1]
                            Rid_sendRobotTM.append(rid)
                        else:
                            flag_start = True
                            for cond in robotTM_scond[rid][start_idx]:
                                if self.PlanExecutedIdx[cond[0]][0] < cond[1][0] or self.PlanExecutedIdx[cond[0]][1] < \
                                        cond[1][1]:
                                    flag_start = False
                            if flag_start:
                                # robotTM[rid] = copy.deepcopy(robotTM_set[rid][start_idx])
                                # self.robotTM[rid] = copy.deepcopy(robotTM[rid])
                                robotTM[rid] = copy.copy(robotTM_set[rid][start_idx])
                                self.robotTM[rid] = copy.copy(robotTM[rid])
                                self.PlanExecutedIdx[rid] = [start_idx, -1]
                                Rid_sendRobotTM.append(rid)
            elif self.Flag_terminate[rid] == 0:
                if self.robotTM[rid] != []:
                    if (vid[0] == self.robotTM[rid][0]) and (vid[1] == self.robotTM[rid][0]):
                        temp_path = self.robotTM[rid]
                        del temp_path[0]
                        self.robotTM[rid] = temp_path
            # print(rid, robotTM[rid], robotTM_set[rid], self.PlanExecutedIdx[rid])
            # print(self.robotGoal)
            test += 1
            print(rid, "execute", self.PlanExecutedIdx[rid])
        print("Iteration TEST : ", test)
        return Rid_sendRobotTM

    ### BACKUP ###
    def update_robot_TM_(self, robot_pose):  # call when the robot position changes - # TODO
        # robot_pose ={robot_id: [vertex, vertex], ...}
        Rid_sendRobotTM = []  # list of robot ids, of which robot TM a navigation controller should send
        # print("Robot TM 2:    ", self.robotTM)
        # check whether the current TM command is executed
        for rid, vid in robot_pose.items():
            self.robotPose[rid] = vid
            if self.robotGoal[rid] != -1:
                if self.robotTM[rid] != []:  # check plan execution
                    compare_nodes = [[self.robotTM[rid][0]] * 2]
                    if len(self.robotTM[rid]) > 1:
                        compare_nodes.append(self.robotTM[rid][0:2])
                        compare_nodes.append([self.robotTM[rid][1], self.robotTM[rid][0]])
                    # if vid in [[self.robotTM[rid][0]]*2]:
                    if (vid[0] == self.robotTM[rid][0]) and (vid[1] == self.robotTM[rid][0]):
                        self.PlanExecutedIdx[rid][1] = self.PlanExecutedIdx[rid][1] + 1

                    if (vid[0] != self.robotTM[rid][0]) and (vid[0] == vid[1]):
                        vidx = self.robotTM[rid].index(vid[0])
                        self.PlanExecutedIdx[rid][1] = self.PlanExecutedIdx[rid][1] + vidx + 1
                else:
                    if (vid[0] == self.robotGoal[rid]) and (vid[1] == self.robotGoal[rid]):
                        self.Flag_terminate[rid] = 0
                        self.robotGoal[rid] = -1

                if self.robotTM_set[rid] != []:  # check flag_terminate
                    if self.PlanExecutedIdx[rid] == [len(self.robotTM_set[rid]) - 1,
                                                     len(self.robotTM_set[rid][-1]) - 1]:
                        if vid == [self.robotGoal[rid]] * 2:
                            self.Flag_terminate[rid] = 0
                            self.robotGoal[rid] = -1
                        # else:
                        #     self.Flag_terminate[rid] = 1
                else:
                    # if vid == [self.robotGoal[rid]]*2:
                    if (vid[0] == self.robotGoal[rid]) and (vid[1] == self.robotGoal[rid]):
                        self.Flag_terminate[rid] = 0
                        self.robotGoal[rid] = -1
                    # else:
                    #     self.Flag_terminate[rid] = 1
        # Update the robot TM
        # print("AMR_TOW1", robot_pose["AMR_TOW1"], self.robotGoal["AMR_TOW1"], self.Flag_terminate["AMR_TOW1"])
        # print("AMR_TOW1", " TM", self.robotTM["AMR_TOW1"])
        # print("AMR_TOW1 ", "TM_set ", self.robotTM_set["AMR_TOW1"])
        # print("AMR_TOW1 ", "ExecutedIdx", self.PlanExecutedIdx["AMR_TOW1"])
        # print("AMR_TOW1 ", "scond ", self.robotTM_scond["AMR_TOW1"])
        # print("AMR_TOW2", robot_pose["AMR_TOW2"], self.robotGoal["AMR_TOW2"], self.Flag_terminate["AMR_TOW2"])
        # print("AMR_TOW2", " TM", self.robotTM["AMR_TOW2"])
        # print("AMR_TOW2 ", "TM_set ", self.robotTM_set["AMR_TOW2"])
        # print("AMR_TOW2 ", "ExecutedIdx", self.PlanExecutedIdx["AMR_TOW2"])
        # print("AMR_TOW2 ", "scond ", self.robotTM_scond["AMR_TOW2"])
        for rid, vid in robot_pose.items():
            if self.Flag_terminate[rid] == -1:
                if self.robotTM_set[rid] != []:
                    if (self.robotTM[rid] != []) and (self.PlanExecutedIdx[rid][1] != -1):  # update robotTM
                        # if vid in [[self.robotTM[rid][0]]*2]: # if a robot arrives at self.robotTM[rid][0]
                        if (vid[0] == self.robotTM[rid][0]) and (vid[1] == self.robotTM[rid][0]):
                            temp_path = self.robotTM[rid]
                            del temp_path[0]
                            self.robotTM[rid] = temp_path
                            # self.robotTM[rid].pop(0)
                        if (vid[0] != self.robotTM[rid][0]) and (vid[0] == vid[1]):
                            vidx = self.robotTM[rid].index(vid[0])
                            del self.robotTM[rid][:vidx + 1]
                    else:  # allocate a new robotTM
                        start_idx = self.PlanExecutedIdx[rid][0] + 1
                        if self.robotTM_scond[rid][start_idx] == []:  # no condition
                            self.robotTM[rid] = copy.copy(self.robotTM_set[rid][start_idx])
                            self.PlanExecutedIdx[rid] = [start_idx, -1]
                            # send the command
                            Rid_sendRobotTM.append(rid)
                        else:  # check condition
                            flag_start = True
                            for cond in self.robotTM_scond[rid][start_idx]:
                                if self.PlanExecutedIdx[cond[0]][0] < cond[1][0] or self.PlanExecutedIdx[cond[0]][1] < \
                                        cond[1][1]:
                                    flag_start = False
                            # for cond in self.robotTM_scond[rid][start_idx][0]:
                            #     if self.PlanExecutedIdx[cond[0]][0] < cond[1][0] or self.PlanExecutedIdx[cond[0]][1] < cond[1][1]:
                            #         flag_start = False
                            if flag_start:
                                self.robotTM[rid] = copy.copy(self.robotTM_set[rid][start_idx])
                                self.PlanExecutedIdx[rid] = [start_idx, -1]
                                # send the command
                                Rid_sendRobotTM.append(rid)
                        # if rid=="AMR_TOW1" or rid=="AMR_TOW2":
                        #     print(rid, "start_idx: ", start_idx, "executedidx: ", self.PlanExecutedIdx[rid])
            elif self.Flag_terminate[rid] == 0:
                if self.robotTM[rid] != []:
                    if (vid[0] == self.robotTM[rid][0]) and (vid[1] == self.robotTM[rid][0]):
                        temp_path = self.robotTM[rid]
                        del temp_path[0]
                        self.robotTM[rid] = temp_path
                        # self.robotTM[rid].pop(0)

        return Rid_sendRobotTM

    def send_RobotTM(self, robotid, robotTM):  # send command to RobotTM
        print("send a command to RobotTM: ", robotid, robotTM)

    def update_start_goal_collision(self, robot_ids):  # execute when the robot TM allocates a goal to each robot
        # goals: {robot_id: goal vertex, ....}, robot_pose = {id, [vertex, vertex]}

        for rid in robot_ids:
            if self.robotTM[rid] != []:  # the robot is executing the plan
                self.robotTM[rid] = [self.robotTM[rid][0]]
                self.robotStart[rid] = self.robotTM[rid][0]
                self.robotTM_set[rid] = [self.robotTM[rid]]  # [robotTM, robotTM, robotTM, ...]
                self.PlanExecutedIdx[rid] = [0, -1]
                self.robotTM_scond[rid] = [[]]  # start condition of robotTM


if __name__ == "__main__":
    AMR_IDs = ['AMRLIFT0', 'AMRLIFT1', 'AMRTOW0', 'AMRTOW1']
    navcont = NavigationControl(AMR_IDs)
    multipaths = {'AMRLIFT0': [1, 2, 4, 6, 5], 'AMRLIFT1': [3, 3, 2, 4, 6, 7, 7, 8], 'AMRTOW0': [], 'AMRTOW1': []}
    navcont.get_multipath_plan(multipaths)

    # test - update extract_TM
    # rid = AMR_IDs[1]
    # for sidx in range(0,6):
    #    navcont.extract_TM(navcont.NavPath[rid][sidx:], navcont.timing[rid][sidx:])

    # test -update_TM
    paths_execute = {'AMRLIFT0': [0, 1, 2, 4, 6, 5, 5, 5], 'AMRLIFT1': [3, 3, 2, 4, 6, 7, 7, 8], 'AMRTOW0': [1] * 10,
                     'AMRTOW1': [2] * 10}
    for t in range(0, 7):
        robot_pose = {}
        for key, val in paths_execute.items():
            robot_pose[key] = [val[t]] * 2
        print(paths_execute['AMRLIFT0'][t], paths_execute['AMRLIFT1'][t])
        navcont.update_robot_TM(robot_pose)

    print("done")

