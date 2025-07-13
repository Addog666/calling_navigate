import time
from Navigator import Navigator
import math

viz = Navigator('localhost')

goal_xy = [(-3.98506498336792, 8.578082084655762), (-3.090315341949463, 9.38940143585205)]
goal_yaw = [-1.60, 1.55]

i = 1
while True:
    x0, y0 = goal_xy[i]
    yaw0 = goal_yaw[i]
    while True:
        # 启动连接并发送指令
        x, y = viz.robot_pose
        yaw = viz.robot_yaw
        dis = (x - x0) ** 2 + (y - y0) ** 2
        delta = (yaw - yaw0) % (2 * math.pi)  # 0 ~ 2pi之间
        if delta > math.pi:
            delta = 2 * math.pi - delta
        print("delta", delta)
        if abs(delta) > 0.3:
            viz.move_robot(0.0, 0.0, 0.2)  # 掉头
        elif dis > 0.1:
            viz.move_robot(0.2, 0.0, 0.0)  # 前进
        else:
            print("reached")
            if i == 0:
                i = 1
            else:
                i = 0
            time.sleep(10.0)
            break
