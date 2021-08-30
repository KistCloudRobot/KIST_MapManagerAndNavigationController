import sys, os, io

# RobotInfo class has a robot information given by MOS

class CallInfo:
    def __init__(self):
        self.timestamp = -1
        self.vertex = [-1, -1]
        self.gl = ""

    def parsing_gl_to_info(self, gl): # convert gl to RobotInfo:  gl may string
        self.gl = gl




