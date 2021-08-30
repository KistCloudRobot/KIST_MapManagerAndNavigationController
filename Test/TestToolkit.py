from DataType.CallInfo import CallInfo
import matplotlib.pyplot as plt

# define functions draw
def draw(map, smap, robots):
    draw_set={'AMRLIFT0':'ro','AMRLIFT1': 'bo', 'AMRTOW0':'r^', 'AMRTOW1':'b^'}
    draw_set_s = {'AMRLIFT0': 'bo', 'AMRLIFT1': 'ro', 'AMRTOW0': 'b^', 'AMRTOW1': 'r^'}

    plt.cla()
    # draw the plan
    for rid, path in smap.Path_AMR_TOW.items():
        x, y = path_to_pos(path=path, map = map)
        plt.plot(x, y, 'o-y', linewidth= 5, markersize = 10)

        # test fine trajectories
        plan_traj = smap.generate_traj_from_plan(init=smap.AMR_TOW[rid]['pos'][-1].copy(), vel=0.8, delT = 0.1, T=3, plan=path.copy())
        plt.plot(plan_traj['x'],plan_traj['y'], 'o-g', linewidth=3, markersize=3)

    for rid, path in smap.Path_AMR_LIFT.items():
        x, y = path_to_pos(path=path, map=map)
        plt.plot(x, y, 'o-y', linewidth=5, markersize = 10)

        # test fine trajectories
        plan_traj = smap.generate_traj_from_plan(init=smap.AMR_LIFT[rid]['pos'][-1].copy(), vel=0.8, delT = 0.1, T=3, plan=path.copy())
        plt.plot(plan_traj['x'], plan_traj['y'], 'o-g', linewidth=3, markersize=3)

    map.draw_map()
    for rid, robot in robots.items():
        # draw true value
        plt.plot(robot.x,robot.y, draw_set[rid],markersize=12)

        # draw the semantic map
        if rid in smap.AMR_LIFT_IDs:
            plt.plot(smap.AMR_LIFT[rid]['pos'][-1][0],smap.AMR_LIFT[rid]['pos'][-1][1], draw_set_s[rid], markersize=8)
            #print("RobotVertex: ", rid, smap.AMR_LIFT[rid]['vertex'][-1])
        elif rid in smap.AMR_TOW_IDs:
            plt.plot(smap.AMR_TOW[rid]['pos'][-1][0], smap.AMR_TOW[rid]['pos'][-1][1], draw_set_s[rid], markersize=8)
            #print("RobotVertex: ", rid, smap.AMR_TOW[rid]['vertex'][-1])


    for id, val in smap.RACK_TOW.items():
        plt.plot(val['pos'][-1][0], val['pos'][-1][1], 's',color='cyan', markersize=5)

    for id, val in smap.RACK_LIFT.items():
        plt.plot(val['pos'][-1][0], val['pos'][-1][1], 's',color='cyan', markersize=5)

    for id, val in smap.CARGO.items():
        plt.plot(val['pos'][- 1][0], val['pos'][-1][1], 'd',color='black', markersize=3)

    plt.pause(0.03)

def get_callinfo(t, vertex):
    info = CallInfo()
    info.vertex = [vertex, vertex]
    info.timestamp = t
    return info

def path_to_pos(path, map): # path: the sequence of vertexes
    x = []
    y = []
    for v in path:
        x.append(map.VertexPos[v][0])
        y.append(map.VertexPos[v][2])
    return x, y
