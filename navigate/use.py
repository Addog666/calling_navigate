import matplotlib.pyplot as plt
import time
from Navigator import Navigator
import matplotlib

matplotlib.use('TkAgg')

viz = Navigator('10.28.11.22')

# 等待获取地图
print("Waiting for map...")
while True:
    map_img = viz.get_map()
    if map_img is not None:
        print("Map received.")
        break
    time.sleep(0.5)

# 设置目标点
start_px = [(993.000, 950.000), (996.000, 1005.000)]
orient_px = [(992.000, 950.000), (997.000, 1005.000)]

i = 0
viz.set_goal(start_px[i], orient_px[i])

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

        if viz.is_goal_reached(threshold_distance=0.3, threshold_angle=0.3):
            print("✅ Robot reached the goal!")
            time.sleep(5.0)
            if i == 0:
                i = 1
            else:
                i = 0
            viz.set_goal(start_px[i], orient_px[i])
    else:
        time.sleep(0.1)
