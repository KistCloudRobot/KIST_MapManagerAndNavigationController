import numpy as np

class VertexCloudlet:
    T = [0, 0.1, 0.2] # time index of map data
    numT = len(T)

    def __init__(self, x=np.inf, y=np.inf, objs=[], err_code=[], robot_stop={}, robot_collision={}, err_freq={}):
        self.x = x # static: x position of the vertex
        self.y = y # static: y position of the vertex
        self.objs = objs # the list of object ids at the vertex ex)[ROBOT01, RACK01]
        self.err_code = err_code # the list of error code
        self.robot_stop = robot_stop # the probability that a robot stops at the vertex {Robotid: [probability, time]}
        self.robot_collision = robot_collision # the probability that collision occurs {[object id, ): [prob, time]}
        self.err_freq = err_freq # {Error_Code: frequency}
        self.door_id = 0


