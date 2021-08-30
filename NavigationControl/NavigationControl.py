class NavigationControl:
    def __init__(self, AMR_IDs,  AMR_LIFT_IDs, AMR_TOW_IDs):
        # initialize
        self.AMR_IDs = AMR_IDs
        self.AMR_TOW_IDs = AMR_TOW_IDs
        self.AMR_LIFT_IDs = AMR_LIFT_IDs

        # path given by multi-robot path finder
        #self.NavPath = {}
        #self.timing = {} # execution timing
        #for id in self.AMR_IDs:
        #    self.NavPath[id] = []
        #    self.timing[id] = [] # if NavPath[rid1][idx1] == NavPath[rid2][idx2] and idx2<idx1 : timing[id]=[[rid2, time_idx2, NavPath[rid1][idx1]], ...]

        # command for RobotTM
        self.robotTM = {} # the current command for RobotTM
        self.robotTM_set = {} # full sequence of commands for RobotTM
        self.robotTM_scond = {} # start condition of robotTM
        self.robotGoal = {} # New: The goal of each robot
        self.robotStart = {} # New: The goal of each robot

        for id in self.AMR_IDs:
            self.robotTM[id] = [] # a sequence of vertices
            self.robotTM_set[id] = [] # [robotTM, robotTM, robotTM, ...]
            self.robotTM_scond[id] = [] # start condition of robotTM
            self.robotGoal[id] = -1

        self.PlanExecutedIdx = {}
        self.Flag_terminate = {} # -1: Not terminated, 0: Success Terminate 1: Fail Terminate
        for id in self.AMR_IDs:
            self.PlanExecutedIdx[id] = [-1, -1] # [i,j] Save the last index of NavPath[i][j] which the robot follows
            self.Flag_terminate[id] = 0
    # New
    def allocate_goal(self, goals, robot_pose): # execute when the robot TM allocates a goal to each robot
        # goals: {robot_id: goal vertex, ....}, robot_pose = {id, [vertex, vertex]}

        Rid_robotTM = [] # the list of robot ids that has an updated robotTM
        Rid_replan = [] # the list of robot ids for replanning
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
            if self.robotGoal[rid] != -1: # the robot has a navigation job => initialize
                Rid_replan.append(rid) # require replanning
                if rid in goals.keys(): # got new job
                    self.robotGoal[rid] = goals[rid]

                if self.robotTM[rid] != []: # the robot is executing the plan # TODO: test
                    self.robotTM[rid] = [self.robotTM[rid][0]]
                    Rid_robotTM.append(rid)
                    self.robotStart[rid] = self.robotTM[rid][0]
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
                if rid in goals.keys(): # get a new job
                    Rid_replan.append(rid)
                    self.robotStart[rid] = robot_pose[rid][0]
                    self.robotGoal[rid] = goals[rid]

        return Rid_replan, Rid_robotTM

    def get_multipath_plan(self, multipaths): # multipath: MultiPath type
        # initialize PlanExecutedIdx
        for id in multipaths.keys():
            self.robotTM[id] = []  # a sequence of vertices
            self.robotTM_set[id] = []  # [robotTM, robotTM, robotTM, ...]
            self.robotTM_scond[id] = []  # start condition of robotTM
            self.PlanExecutedIdx[id] = [-1, -1] # [i,j] Save the last index of NavPath[i][j] which the robot follows
            self.Flag_terminate[id] = -1

        start_condition = {}
        for key in multipaths.keys():
            start_condition[key] = []
        # Analyze timing
        for rid, path in multipaths.items():

            for idx in range(0, len(path)): # compare
                scond = []
                rid_list = list(multipaths.keys())
                rid_list.remove(rid)
                for rid2 in rid_list:
                    for idx2 in range(min(idx,len(multipaths[rid2])-1), -1, -1):
                        if path[idx] == multipaths[rid2][idx2]:
                            scond.append([rid2, idx2])

                start_condition[rid].append(scond)

        # Split navigation paths to robotTM
        robotTM_mapping_set = {}
        scondTM_set = {}
        for rid, path in multipaths.items():
            robotTM_seq = []
            robotTM_mapping = [] # save index of robotTM_set corresponding to each element in path
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
                        t1 = t1+1
                        t2 = 0
                        robotTM  = [path[ii]]
                        scond.append(start_condition[rid][ii])

                    robotTM_mapping.append([t1,t2])

                robotTM_seq.append(robotTM)

            self.robotTM_set[rid] = robotTM_seq.copy()
            robotTM_mapping_set[rid] = robotTM_mapping
            scondTM_set[rid] = scond

        # find start condnition for TM
        scond_TM_translated = {}
        for rid in multipaths.keys():
            scond = []
            for tt in range(0, len(scondTM_set[rid])):
                if scondTM_set[rid][tt] !=[]:
                    cond_cur = []
                    for cond in scondTM_set[rid][tt]:
                        cond_cur.append([cond[0], robotTM_mapping_set[cond[0]][cond[1]]])

                    scond.append(cond_cur)
                else:
                    scond.append([])

#            scond_TM_translated[rid] = scond
            self.robotTM_scond[rid] = scond

#        self.robotTM_scond = scond_TM_translated

    def update_robot_TM(self, robot_pose): # call when the robot position changes - # TODO
        # robot_pose ={robot_id: [vertex, vertex], ...}
        Rid_sendRobotTM = [] # list of robot ids, of which robot TM a navigation controller should send
        #print("Robot TM 2:    ", self.robotTM)

        # check whether the current TM command is executed
        for rid, vid in robot_pose.items():
            if self.robotGoal[rid] !=-1:
                if self.robotTM[rid] != []: # check plan execution
                    print(vid)
                    compare_nodes = [[self.robotTM[rid][0]]*2]
                    if len(self.robotTM[rid])>1:
                        compare_nodes.append(self.robotTM[rid][0:2])
                        compare_nodes.append([self.robotTM[rid][1], self.robotTM[rid][0]])
                    if vid in [[self.robotTM[rid][0]]*2]:
                        self.PlanExecutedIdx[rid][1] = self.PlanExecutedIdx[rid][1] + 1
                if self.robotTM_set[rid] != []: # check flag_terminate
                    if self.PlanExecutedIdx[rid] == [len(self.robotTM_set[rid])-1, len(self.robotTM_set[rid][-1])-1]:
                        if vid == [self.robotGoal[rid]]*2:
                            self.Flag_terminate[rid] = 0
                            self.robotGoal[rid] = -1
                        else:
                            self.Flag_terminate[rid] = 1

                else:
                    if vid == [self.robotGoal[rid]] * 2:
                        self.Flag_terminate[rid] = 0
                        self.robotGoal[rid] = -1
                    else:
                        self.Flag_terminate[rid] = 1

        # Update the robot TM
        for rid, vid in robot_pose.items():
            if self.Flag_terminate[rid] == -1:
                if self.robotTM_set[rid] !=[]:
                    if self.robotTM[rid] !=[]: # update robotTM
                        if vid in [[self.robotTM[rid][0]]*2]: # if a robot arrives at self.robotTM[rid][0]
                            self.robotTM[rid].pop(0)
                    else: # allocate a new robotTM
                        start_idx = self.PlanExecutedIdx[rid][0]+1
                        if self.robotTM_scond[rid][start_idx] == []: # no condition
                            self.robotTM[rid] = self.robotTM_set[rid][start_idx].copy()
                            self.PlanExecutedIdx[rid] = [start_idx, -1]
                            # send the command
                            Rid_sendRobotTM.append(rid)
                        else: # check condition
                            flag_start = True
                            for cond in self.robotTM_scond[rid][start_idx]:
                                if self.PlanExecutedIdx[cond[0]][0] < cond[1][0] or self.PlanExecutedIdx[cond[0]][1] < cond[1][1]:
                                    flag_start = False
                            if flag_start:
                                self.robotTM[rid] = self.robotTM_set[rid][start_idx].copy()
                                self.PlanExecutedIdx[rid] = [start_idx, -1]
                                # send the command
                                Rid_sendRobotTM.append(rid)

        #print("PlanExecuted ", self.PlanExecutedIdx)
        #print("Robot TM 2:    ",self.robotTM)
        #print("robotGoal: {}".format(self.robotGoal))
        print("Flag:{}".format(self.Flag_terminate))

        return Rid_sendRobotTM

    def send_RobotTM(self, robotid, robotTM): # send command to RobotTM
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
    multipaths = {'AMRLIFT0': [1,2,4,6,5], 'AMRLIFT1': [3,3,2,4,6,7,7,8], 'AMRTOW0':[], 'AMRTOW1':[]}
    navcont.get_multipath_plan(multipaths)

    # test - update extract_TM
    #rid = AMR_IDs[1]
    #for sidx in range(0,6):
    #    navcont.extract_TM(navcont.NavPath[rid][sidx:], navcont.timing[rid][sidx:])

    # test -update_TM
    paths_execute =  {'AMRLIFT0': [0,1,2,4,6,5,5,5], 'AMRLIFT1': [3,3,2,4,6,7,7,8], 'AMRTOW0':[1]*10, 'AMRTOW1':[2]*10}
    for t in range(0,7):
        robot_pose = {}
        for key, val in paths_execute.items():
            robot_pose[key] = [val[t]]*2
        print(paths_execute['AMRLIFT0'][t], paths_execute['AMRLIFT1'][t])
        navcont.update_robot_TM(robot_pose)

    print("done")

