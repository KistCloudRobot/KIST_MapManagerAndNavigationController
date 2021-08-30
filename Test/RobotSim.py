from MapManagement import MapMOS
from DataType.RobotInfo import RobotInfo
import numpy as np

# A robot agent in simulation
class RobotSim:
    def __init__(self, id, x,y, static_map):

        self.VEL = 0.800  # m
        self.NOISE_SIG = 0.0 # m
        self.T_LOAD = 1 # sec
        self.T_UNLOAD = 1 # sec

        self.id = id
        self.x = x
        self.y = y
        self.plan = []  # [type, value], ['move', the list of vertex), 'load'], ['unload']]
        self.t_req_load = self.T_LOAD # left time to laod the object
        self.t_req_unload = 0 #
        self.status = 'none' # ['none','ing','done'] if a robot is executing a plan 2: done
        self.loading = 0 # whether an object is loaded or not
        self.static_map = static_map


    def move_to_node(self, x,y, node, t): # move to an adjacent node for time t. return left time and x,y position
        goal_pos = [self.static_map.VertexPos[node][0], self.static_map.VertexPos[node][2]] # goal position
        goal_dist = np.sqrt((goal_pos[0]-x)**2 + (goal_pos[1]-y)**2) # distance from the current pos to goal position

        t_req = goal_dist/ self.VEL # time to arrive the node

        if t >= t_req:
            if self.static_map.VertexType == 0:
                xnext = goal_pos[0] + np.random.normal(scale=self.NOISE_SIG)
                ynext = goal_pos[1] + np.random.normal(scale=self.NOISE_SIG)
            else:
                xnext = goal_pos[0]
                ynext = goal_pos[1]

        else:
            xnext = x + (goal_pos[0] - x) / goal_dist * self.VEL*t + np.random.normal(scale=self.NOISE_SIG)
            ynext = y + (goal_pos[1] - y) / goal_dist * self.VEL*t + np.random.normal(scale=self.NOISE_SIG)

        t_left =  t - t_req

        return xnext, ynext, t_left

    def execute_plan(self, t): # Execute the allocated plan for time t
        if self.plan !=[]:
            if self.plan[0] == 'move':
                while t > 0 and self.plan != []:
                    self.x, self.y, t = self.move_to_node(self.x, self.y, self.plan[1][0], t)
                    if t > 0:
                        self.plan[1].pop(0)

                    if self.plan[1] == []:
                        self.plan = []
                        self.status = 'done'

            elif self.plan[0] == 'load':
                print(self.t_req_load)
                if self.t_req_load > self.T_LOAD:
                    self.t_req_load = self.t_req_load - self.T_LOAD


                else:
                    self.t_req_load = 0
                    self.t_req_unload = self.T_UNLOAD
                    self.loading = 1
                    self.plan = []
                    self.status = 'done'

            elif self.plan[0] == 'unload':
                if self.t_req_unload > self.T_UNLOAD:
                    self.t_req_unload = self.t_req_unload - self.T_UNLOAD

                else:
                    self.t_req_load = self.T_LOAD
                    self.t_req_unload = 0
                    self.loading = 0
                    self.plan = []
                    self.status = 'done'

    def clear_plan(self):
        self.plan = []

    def insert_plan(self, plan):
        self.plan = plan.copy()
        self.status = 'ing'

    def print_data(self):
        print(self.id, self.x, self.y,self.status, self.loading, self.plan)

    def get_Robotinfo(self, t):
        robotinfo = RobotInfo()
        robotinfo.id = self.id
        robotinfo.timestamp = t
        robotinfo.pos = [self.x, self.y]
        robotinfo.load = self.loading

        return robotinfo






