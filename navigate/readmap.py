import roslibpy
import numpy as np
import matplotlib.pyplot as plt
import time
import math
import matplotlib

matplotlib.use('TkAgg')  # 切换到 TkAgg 后端

# 全局变量
fig, ax = plt.subplots()
img_plot = None
map_info = None
clicks = []
base_map_rgb = None  # 原始地图副本
display_map_rgb = None  # 当前显示地图（带路径）
map_resolution = 0.0
map_origin = None

# ROS连接
client = roslibpy.Ros(host='10.28.11.22', port=9090)
client.run()

goal_publisher = roslibpy.Topic(client, '/goal_pose', 'geometry_msgs/msg/PoseStamped')
goal_publisher.advertise()


# 鼠标点击设置目标与朝向
def on_click(event):
    global clicks

    if event.inaxes and map_info is not None:
        pixel_x, pixel_y = int(event.xdata), int(event.ydata)
        print(f"Clicked image coordinate: ({pixel_x:.3f}, {pixel_y:.3f})")
        map_x = map_origin['x'] + pixel_x * map_resolution
        map_y = map_origin['y'] + pixel_y * map_resolution

        clicks.append((map_x, map_y))
        print(f"Clicked map coordinate: ({map_x:.3f}, {map_y:.3f})")

        if len(clicks) == 2:
            x1, y1 = clicks[0]
            x2, y2 = clicks[1]
            dx = x2 - x1
            dy = y2 - y1
            yaw = math.atan2(dy, dx)
            qz = math.sin(yaw / 2)
            qw = math.cos(yaw / 2)

            goal_msg = {
                'header': {
                    'frame_id': 'map',
                    'stamp': {
                        'sec': int(time.time()),
                        'nanosec': int((time.time() % 1) * 1e9)
                    }
                },
                'pose': {
                    'position': {'x': x1, 'y': y1, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': qz, 'w': qw}
                }
            }

            print("Publishing goal to /goal_pose:", goal_msg)
            goal_publisher.publish(roslibpy.Message(goal_msg))
            clicks = []


# 接收地图并生成三通道RGB地图
def map_callback(message):
    global img_plot, map_info, base_map_rgb, display_map_rgb, map_resolution, map_origin

    map_info = message['info']
    map_resolution = map_info['resolution']
    map_origin = map_info['origin']['position']
    width = map_info['width']
    height = map_info['height']
    data = np.array(message['data']).reshape((height, width))

    # 灰度地图到RGB图像
    gray = np.zeros((height, width), dtype=np.uint8)
    gray[data == 0] = 255  # 空闲区域
    gray[data == 100] = 0  # 障碍物
    gray[data == -1] = 127  # 未知区域
    base_map_rgb = np.stack([gray] * 3, axis=-1)  # 转为三通道

    display_map_rgb = base_map_rgb.copy()

    if img_plot is None:
        img_plot = ax.imshow(display_map_rgb, origin='lower')
        plt.title("Path drawn directly on RGB map (click to set goal)")
        fig.canvas.mpl_connect('button_press_event', on_click)
        plt.ion()
        plt.show()
    else:
        img_plot.set_data(display_map_rgb)
        fig.canvas.draw_idle()


# 处理路径并画在RGB地图图像上
def path_callback(message):
    global display_map_rgb, base_map_rgb, map_resolution, map_origin

    if base_map_rgb is None or map_origin is None:
        return

    display_map_rgb = base_map_rgb.copy()

    for pose in message['poses']:
        x = pose['pose']['position']['x']
        y = pose['pose']['position']['y']
        px = int((x - map_origin['x']) / map_resolution)
        py = int((y - map_origin['y']) / map_resolution)

        # 检查边界，画红色像素
        if 0 <= px < display_map_rgb.shape[1] and 0 <= py < display_map_rgb.shape[0]:
            display_map_rgb[py, px] = [255, 0, 0]

    if img_plot is not None:
        img_plot.set_data(display_map_rgb)
        fig.canvas.draw_idle()


# ROS 订阅
map_sub = roslibpy.Topic(client, '/map', 'nav_msgs/msg/OccupancyGrid')
map_sub.subscribe(map_callback)

path_sub = roslibpy.Topic(client, '/plan', 'nav_msgs/Path')
path_sub.subscribe(path_callback)

# 主循环
try:
    while client.is_connected:
        plt.pause(0.1)
except KeyboardInterrupt:
    print("Shutting down.")
    map_sub.unsubscribe()
    path_sub.unsubscribe()
    goal_publisher.unadvertise()
    client.terminate()
