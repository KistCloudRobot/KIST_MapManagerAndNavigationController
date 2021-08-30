from Test.RobotSim import RobotSim
from MapManagement.MapMOS import MapMOS
from MapManagement.MapCloudlet import MapCloudlet
from DataType.RobotInfo import RobotInfo
from DataType.CallInfo import CallInfo
import matplotlib.pyplot as plt
from NavigationControl.NavigationControl import *
from Test.TestToolkit import *


# Setting
map_file = "../data/map_cloud.txt"
MAP = MapMOS(map_file)
AMR_LIFT_IDs = ['AMRLIFT0', 'AMRLIFT1']
AMR_TOW_IDs = ['AMRTOW0', 'AMRTOW1']

# RobotPlan Format:{robotID: [('move', [1,2,3,.. ]), ('load'), ('unload'), ....]} 'call': wait until call function occurs
RobotPlan = {'AMRLIFT0': [['move',[220,7]], ['load'], ['move',[220,219,21]], ['unload'], ['move',[219,220]]],
             'AMRLIFT1':[['move',[205,206,207,3]], ['load'],['move',[207,208,209]]],
             'AMRTOW0': [['call'], ['move',[233,234,22]], ['load'], ['move', [234,235,236,237,238,239,20]],
                         ['call'],['move',[239,238]]],
             'AMRTOW1': [['move',[207,206]]]}
#MultiPath = {'AMRLIFT0': [219, 220, 7], 'AMRLIFT1': [222,221,220,219]}
MultiPath1 = {'AMRLIFT0': [219, 220, 7], 'AMRTOW0':[234,235,236,237,238,239]}
MultiPath2 = {'AMRLIFT0': [219, 220, 7], 'AMRLIFT1': [221,221,220,219,218]}
Goal1 = {'AMRLIFT0': 7, 'AMRTOW0':239}
Goal2 = {'AMRLIFT1':218}
# RobotInit
AMR_LIFT_init = {'AMRLIFT0':218, 'AMRLIFT1':223} # 218, 222
AMR_TOW_init = {'AMRTOW0':230, 'AMRTOW1':3}
init_poses = {'AMRLIFT0':[218,218], 'AMRLIFT1':[223,223], 'AMRTOW0': [22,22]}
init_poses2 = {'AMRLIFT0':[218,219], 'AMRLIFT1':[223,223], 'AMRTOW0':[22,22]}
#RobotInitPos = {}
#for key, vertex in RobotInit.items():
#    RobotInitPos[key] = [MAP.VertexPos[vertex][0], MAP.VertexPos[vertex][2]]

# CargoInit
Cargo_init = [7,3]
# RACK
Rack_LIFT_init = {'RACKLIFT0':18, 'RACKLIFT1':19, 'RACKLIFT2':7, 'RACKLIFT3':3}
Rack_TOW_init = {'RACKTOW0':22, 'RACKTOW1':13}

# Door_init
Door_init = {'Door0':0}


T_DEL = 0.2 # sec time interval for one step

# Initialize
smap = MapCloudlet(map_file,AMR_TOW_init = AMR_TOW_init, AMR_LIFT_init=AMR_LIFT_init,
                   RACK_TOW_init=Rack_TOW_init, RACK_LIFT_init=Rack_LIFT_init,
                   Door_init = Door_init, t_init=0) # semantic map
NavCont = NavigationControl(AMR_LIFT_IDs + AMR_TOW_IDs, AMR_LIFT_IDs, AMR_TOW_IDs)
NavCont.allocate_goal(Goal1, robot_pose=init_poses)
NavCont.get_multipath_plan(MultiPath1) # allocate multi paths

robots = {}
for rid, node in AMR_LIFT_init.items():
    robots[rid] = RobotSim(rid, MAP.VertexPos[node][0], MAP.VertexPos[node][2], MAP)
for rid, node in AMR_TOW_init.items():
    robots[rid] = RobotSim(rid, MAP.VertexPos[node][0], MAP.VertexPos[node][2], MAP)

FLAG_RUN = True
timestamp = 0

smap.call_LIFT(get_callinfo(timestamp, Cargo_init[0]))
smap.call_LIFT(get_callinfo(timestamp, Cargo_init[1]))
#smap.add_RACK_TOW(timestamp,RackTOWInit[0])

def print_vertex_TM(smap, NavCont):
    for key in smap.AMR_LIFT_IDs:
        print("[{}] Vertex: {}, robotTM_left: {}".format(key, smap.AMR_LIFT[key]['vertex'][-1],
                                                         NavCont.robotTM[key]))
    for key in smap.AMR_TOW_IDs:
        print("[{}] Vertex: {}, robotTM_left: {}".format(key, smap.AMR_TOW[key]['vertex'][-1],
                                                         NavCont.robotTM[key]))
# Run
num11 = 0
while FLAG_RUN:
    if num11==2: # The new goal is allocated
        Rid_replan, Rid_robotTM = NavCont.allocate_goal(Goal2, robot_pose=init_poses2)
        for rid in Rid_robotTM:
            robots[rid].insert_plan(['move', NavCont.robotTM[rid].copy()])
            smap.insert_NAV_PLAN(rid, NavCont.robotTM[rid].copy())
            smap.update_NAV_PLAN(rid)

    if num11 == 10:
        NavCont.get_multipath_plan(MultiPath2)  # allocate multi paths
        print(" ")

    num11 = num11+1
    print('----------------------------')
    print("1: ", NavCont.robotTM)
    timestamp = timestamp + T_DEL

    # derive robot_pose_cur
    robot_pose_cur = {}
    for rid in AMR_LIFT_IDs:
        robot_pose_cur[rid] = smap.AMR_LIFT[rid]['vertex'][-1]
    for rid in AMR_TOW_IDs:
        robot_pose_cur[rid] = smap.AMR_TOW[rid]['vertex'][-1]

    # update robot_TM
    TM_update_list = NavCont.update_robot_TM(robot_pose=robot_pose_cur)
    print("2: ", NavCont.robotTM)

    # check if all plans are done
    flag_terminate = True  # check if plans of all robots are done (RobotPlan empty and robots empty)

    for flag_t in NavCont.Flag_terminate.values():
        flag_terminate = flag_terminate and (flag_t == 0)

    # send commands to robotTM
    if not flag_terminate:
        for rid in TM_update_list:
            robots[rid].insert_plan(['move', NavCont.robotTM[rid].copy()])
            smap.insert_NAV_PLAN(rid, NavCont.robotTM[rid].copy())
            smap.update_NAV_PLAN(rid)
    else:
        break
    # Execute plans
    for key in robots.keys():
        robots[key].execute_plan(T_DEL) # execute

        if robots[key].status == 'done':
            robots[key].status = 'none'
    # Update the semantic map
    for key in robots.keys():
        # update the semantic map
        smap.update_MOS_robot_info(robots[key].get_Robotinfo(timestamp))

    #print_vertex_TM(smap, NavCont)

    # Check the collision
    collision_set = smap.detect_collision(T=3)
    if collision_set != []:
        print(collision_set)
        plt.pause(2)

    draw(MAP, smap, robots)








