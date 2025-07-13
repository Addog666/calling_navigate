import time
from Navigator import Navigator

viz = Navigator('192.168.3.32')

time.sleep(5.0)

while True:
    # 世界坐标
    x, y = viz.robot_pose
    yaw = viz.robot_yaw
    print(x, y)
    print(yaw)
    # 像素坐标
    # img_x = (x - viz.map_origin['x']) / viz.map_resolution
    # img_y = (y - viz.map_origin['y']) / viz.map_resolution
    # print(img_x, img_y)
