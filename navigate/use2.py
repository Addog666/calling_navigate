import matplotlib.pyplot as plt
import time
import matplotlib


def move_robot(viz, location):
    matplotlib.use('TkAgg')

    # 等待获取地图
    print("Waiting for map...")
    while True:
        map_img = viz.get_map()
        if map_img is not None:
            print("Map received.")
            break
        time.sleep(0.5)

    if location == "桌子后面":
        # 设置目标点
        goal_xy = [-2.361250877380371, -1.180881381034851]
        goal_yaw = 0.7538740688677131
        viz.set_goal_direct(goal_xy[0], goal_xy[1], goal_yaw)
    elif location == "桌子前面":
        # 设置目标点
        goal_xy = [0.22247804701328278, 0.49352964758872986]
        goal_yaw = -0.7741048622254161
        viz.set_goal_direct(goal_xy[0], goal_xy[1], goal_yaw)
    else:
        return

    # 实时显示路径
    while True:
        if map_img is not None:
            path = viz.get_path()

            map_copy = map_img.copy()
            for px, py in path:
                if 0 <= px < map_copy.shape[1] and 0 <= py < map_copy.shape[0]:
                    map_copy[py, px] = [255, 0, 0]

            plt.clf()
            plt.imshow(map_copy, origin='lower')
            plt.title("Path on Map")
            plt.pause(0.1)

            if viz.is_goal_reached(threshold_distance=0.25, threshold_angle=0.25):
                print("✅ Robot reached the goal!")
                time.sleep(5.0)
                break

        else:
            time.sleep(0.1)
