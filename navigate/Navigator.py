import roslibpy
import numpy as np
import time
import math


class Navigator:
    def __init__(self, ros_address: str, ros_port: int = 9090):
        self.client = roslibpy.Ros(host=ros_address, port=ros_port)
        self.client.run()

        self.goal_publisher = roslibpy.Topic(self.client, '/goal_pose', 'geometry_msgs/msg/PoseStamped')
        self.goal_publisher.advertise()

        self.map_sub = roslibpy.Topic(self.client, '/map', 'nav_msgs/msg/OccupancyGrid')
        self.map_sub.subscribe(self._map_callback)

        self.path_sub = roslibpy.Topic(self.client, '/plan', 'nav_msgs/msg/Path')
        self.path_sub.subscribe(self._path_callback)

        self.cmd_vel_pub = roslibpy.Topic(self.client, '/cmd_vel', 'geometry_msgs/Twist')
        self.cmd_vel_pub.advertise()

        self.cmd_vel_sub = roslibpy.Topic(self.client, '/cmd_vel', 'geometry_msgs/Twist')
        self.cmd_vel_sub.subscribe(self._cmd_vel_callback)

        self.tf_sub = roslibpy.Topic(self.client, '/tf', 'tf2_msgs/msg/TFMessage')
        self.tf_sub.subscribe(self._tf_callback)

        self.map_info = None
        self.map_resolution = 0.0
        self.map_origin = None
        self.base_map_rgb = None
        self.path_pixel_coords = []

        self.robot_pose = None
        self.goal_position = None
        self.goal_yaw = 0.0
        self.robot_yaw = 0.0
        self.twist = None

    # 发布速度
    def move_robot(self, linear_x, liner_y, angular):
        twist_msg = {
            'linear': {'x': linear_x, 'y': liner_y, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': angular}
        }
        self.cmd_vel_pub.publish(twist_msg)

    def _tf_callback(self, message):
        transforms = message['transforms']
        for transform in transforms:
            if transform['header']['frame_id'] == 'map' and transform['child_frame_id'] == 'livox_frame':
                translation = transform['transform']['translation']
                rotation = transform['transform']['rotation']
                self.robot_pose = (translation['x'], translation['y'])

                # 计算 yaw（从四元数）
                qx, qy, qz, qw = rotation['x'], rotation['y'], rotation['z'], rotation['w']
                siny_cosp = 2 * (qw * qz + qx * qy)
                cosy_cosp = 1 - 2 * (qy ** 2 + qz ** 2)
                self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
                break  # 找到一次就退出循环

    def _map_callback(self, message):
        self.map_info = message['info']
        self.map_resolution = self.map_info['resolution']
        self.map_origin = self.map_info['origin']['position']
        width = self.map_info['width']
        height = self.map_info['height']
        data = np.array(message['data']).reshape((height, width))

        gray = np.zeros((height, width), dtype=np.uint8)
        gray[data == 0] = 255
        gray[data == 100] = 0
        gray[data == -1] = 127

        self.base_map_rgb = np.stack([gray] * 3, axis=-1)

    # 订阅全局路径（像素坐标）
    def _path_callback(self, message):
        if self.base_map_rgb is None or self.map_origin is None:
            return

        self.path_pixel_coords = []
        for pose in message['poses']:
            x = pose['pose']['position']['x']
            y = pose['pose']['position']['y']
            px = int((x - self.map_origin['x']) / self.map_resolution)
            py = int((y - self.map_origin['y']) / self.map_resolution)

            if 0 <= px < self.base_map_rgb.shape[1] and 0 <= py < self.base_map_rgb.shape[0]:
                self.path_pixel_coords.append((px, py))

    def _cmd_vel_callback(self, message):
        self.twist = {
            'linear': {
                'x': message['linear']['x'],
                'y': message['linear']['y'],
                'z': message['linear']['z']
            },
            'angular': {
                'x': message['angular']['x'],
                'y': message['angular']['y'],
                'z': message['angular']['z']
            }
        }

    # 设置目标点（像素坐标）
    def set_goal(self, point1, point2):
        if self.map_origin is None:
            print("Map not yet received. Cannot set goal.")
            return

        pixel_x1, pixel_y1 = point1
        pixel_x2, pixel_y2 = point2

        x1 = self.map_origin['x'] + pixel_x1 * self.map_resolution
        y1 = self.map_origin['y'] + pixel_y1 * self.map_resolution
        x2 = self.map_origin['x'] + pixel_x2 * self.map_resolution
        y2 = self.map_origin['y'] + pixel_y2 * self.map_resolution

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

        self.goal_publisher.publish(roslibpy.Message(goal_msg))
        print("Publishing goal to /goal_pose:", goal_msg)
        self.goal_position = (x1, y1)
        self.goal_yaw = yaw

    # 设置目标点（世界坐标）
    def set_goal_direct(self, x, y, yaw):
        if self.map_origin is None:
            print("Map not yet received. Cannot set goal.")
            return

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
                'position': {'x': x, 'y': y, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': qz, 'w': qw}
            }
        }

        self.goal_publisher.publish(roslibpy.Message(goal_msg))
        print("Publishing goal to /goal_pose:", goal_msg)
        self.goal_position = (x, y)
        self.goal_yaw = yaw

    def get_map(self):
        return self.base_map_rgb.copy() if self.base_map_rgb is not None else None

    def get_path(self):
        return self.path_pixel_coords.copy()

    # 到达目标点判断
    def is_goal_reached(self, threshold_distance=0.5, threshold_angle=0.5):
        if self.robot_pose is None or self.goal_position is None or self.robot_yaw is None or self.goal_yaw is None:
            return False

        # 位置判断
        x, y = self.robot_pose
        gx, gy = self.goal_position
        dist = math.hypot(gx - x, gy - y)
        print("dist", dist)
        if dist > threshold_distance:
            return False

        # 角度判断（考虑角度 wrap-around 问题）
        angle_diff = abs(self.robot_yaw - self.goal_yaw)
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Wrap 到 [-pi, pi]
        print("dist", dist)
        print("ang_diff", angle_diff)
        return abs(angle_diff) < threshold_angle

    def shutdown(self):
        self.map_sub.unsubscribe()
        self.path_sub.unsubscribe()
        self.goal_publisher.unadvertise()
        self.cmd_vel_pub.unadvertise()
        self.cmd_vel_sub.unsubscribe()
        self.client.terminate()
