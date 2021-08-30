import sys, os, io

# RobotInfo class has a robot information given by MOS

class RobotInfo:
    def __init__(self):
        self.id = -1
        self.timestamp = -1
        self.pos = -1
        self.vertex = [-1, -1]
        self.load = 0 # 1,0 loaded  or not
        self.gl = ""

    def parsing_gl_to_info(self, gl): # convert gl to RobotInfo:  gl may string
        self.gl = gl




