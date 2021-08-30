from Test.RobotSim import RobotSim
from MapManagement.MapMOS import MapMOS
from MapManagement.MapCloudlet import MapCloudlet
from DataType.RobotInfo import RobotInfo
from DataType.CallInfo import CallInfo
import matplotlib.pyplot as plt
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

# RobotInit
AMR_LIFT_init = {'AMRLIFT0':219, 'AMRLIFT1':204}
AMR_TOW_init = {'AMRTOW0':230, 'AMRTOW1':3}
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


T_DEL = 0.5 # sec time interval for one step

# Initialize
smap = MapCloudlet(map_file,AMR_TOW_init = AMR_TOW_init, AMR_LIFT_init=AMR_LIFT_init,
                   RACK_TOW_init=Rack_TOW_init, RACK_LIFT_init=Rack_LIFT_init,
                   Door_init = Door_init, t_init=0) # semantic map
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
# Run
while FLAG_RUN:
    timestamp = timestamp + T_DEL
    flag_terminate = True  # check if plans of all robots are done (RobotPlan empty and robots empty)

    # insert the plan if the plan is empty
    for rid, plan in RobotPlan.items():
        if plan !=[]:
            if robots[rid].plan == [] and plan[0][0] !='call': # if empty, insert plan and not call
                robots[rid].insert_plan(plan[0].copy())
                if plan[0][0] == 'move': # allocate the plan
                    smap.insert_NAV_PLAN(rid,plan[0][1].copy())
                    #print("insert: NavPlan")
                RobotPlan[rid].pop(0)
                flag_terminate = False


    for robot in robots.values():
        flag_terminate = flag_terminate and robot.plan == []

    FLAG_RUN = not flag_terminate

    #if robots['AMRLIFT0']: #TODO:  execute call functions

    # Execute plans
    print('----------------------------')
    for key in robots.keys():
        #print(key, robots[key].plan)
        robots[key].execute_plan(T_DEL) # execute


        if robots[key].status == 'done':
            robots[key].status = 'none'

    # Update the semantic map
    for key in robots.keys():
        # update the semantic map
        smap.update_MOS_robot_info(robots[key].get_Robotinfo(timestamp))

        # print
        if key in AMR_LIFT_IDs:
            print('path plan ', key, smap.Path_AMR_LIFT[key])
            print("RobotVertex: ", key, smap.AMR_LIFT[key]['vertex'][-1])
        elif key in AMR_TOW_IDs:
            print('path plan ', key, smap.Path_AMR_TOW[key])
            print("RobotVertex: ", key, smap.AMR_TOW[key]['vertex'][-1])

    # Check the collision
    collision_set = smap.detect_collision(T=3)
    if collision_set != []:
        print(collision_set)
        plt.pause(2)

    # Call TOW when tow arrives at the vertex
    #print(smap.AMR_LIFT['AMRLIFT0']['vertex'][-1])
    cargo_id = smap.search_obj_at_vertex(smap.CARGO, [21, 21])
    if cargo_id != -1:
        #tow_id = smap.CARGO[cargo_id]['load_id'][-1][1]
        tow_id = 'AMRTOW0'
        if RobotPlan[tow_id][0] == ['call'] and robots[tow_id].plan == []:
            RobotPlan[tow_id].pop(0)
            print('call_tow')
            info_call = CallInfo()
            info_call.vertex = [22,22]
            smap.call_TOW(info_call)

    # Call removeCargo
    cargo_id = smap.search_obj_at_vertex(smap.CARGO, [20, 20])
    if cargo_id != -1:
        tow_id = 'AMRTOW0'
        if RobotPlan[tow_id][0] == ['call'] and robots[tow_id].plan == []:
            RobotPlan[tow_id].pop(0)
            print('call_remove')
            info_call = CallInfo()
            info_call.vertex = [20,20]
            smap.call_removeCargo(info_call)


    draw(MAP, smap, robots)








