import math

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle, Circle

plt.rcParams['font.sans-serif'] = ['SimHei']  # 设置显示中文字体
plt.rcParams['axes.unicode_minus'] = False  # 设置正常显示符号


class Grid_Map:
    def __init__(self, env, grid_size=0.05, left=0, right=50, bottom=0, top=50):
        """
        初始化栅格地图
        
        参数:
            env: 环境对象，包含障碍物信息
            grid_size: 栅格大小
        """
        self.env = env
        # self.grid_size = env.gridSize
        self.grid_size = grid_size
        self.grid_map = None
        self.width = 0
        self.height = 0
        self.origin_x = 0
        self.origin_y = 0

        self.left = left
        self.right = right
        self.bottom = bottom
        self.top = top

        # 生成栅格地图
        self.generate_grid_map()

    def generate_grid_map(self):
        """生成栅格地图"""
        # 确定地图边界
        # min_x, max_x, min_y, max_y = self._get_map_boundary()
        min_x, max_x, min_y, max_y = self.left, self.right, self.bottom, self.top
        # 计算栅格地图尺寸
        self.width = int((max_x - min_x) / self.grid_size) + 1
        self.height = int((max_y - min_y) / self.grid_size) + 1
        self.origin_x = min_x
        self.origin_y = min_y

        # 初始化栅格地图，0表示空闲，1表示障碍物
        self.grid_map = [[0 for _ in range(self.width)] for _ in range(self.height)]

        # 标记障碍物
        self._mark_obstacles()

    def get_size(self):
        return self.grid_size

    def _get_map_boundary(self):
        """获取地图边界"""
        min_x, max_x = float('inf'), float('-inf')
        min_y, max_y = float('inf'), float('-inf')

        # 检查起点和终点
        min_x = min(min_x, self.env.start[0], self.env.goal[0])
        max_x = max(max_x, self.env.start[0], self.env.goal[0])
        min_y = min(min_y, self.env.start[1], self.env.goal[1])
        max_y = max(max_y, self.env.start[1], self.env.goal[1])

        # 检查矩形障碍物
        for obs in self.env.obs_rectangle:
            x, y, w, h = obs
            min_x = min(min_x, x)
            max_x = max(max_x, x + w)
            min_y = min(min_y, y)
            max_y = max(max_y, y + h)

        # 检查圆形障碍物
        for obs in self.env.obs_circle:
            x, y, r = obs
            min_x = min(min_x, x - r)
            max_x = max(max_x, x + r)
            min_y = min(min_y, y - r)
            max_y = max(max_y, y + r)

        # 添加边界缓冲
        buffer = 1
        min_x -= buffer
        min_y -= buffer
        max_x += buffer
        max_y += buffer
        return min_x, max_x, min_y, max_y

    def _mark_obstacles(self):
        """在栅格地图上标记障碍物"""
        # 标记矩形障碍物
        for obs in self.env.obs_rectangle:
            x, y, w, h = obs
            self._mark_rectangle(x, y, w, h)

        # 标记圆形障碍物
        for obs in self.env.obs_circle:
            x, y, r = obs
            self._mark_circle(x, y, r)

    def _mark_rectangle(self, x, y, w, h):
        """标记矩形障碍物"""
        # 将实际坐标转换为栅格坐标
        grid_x_start = int((x - self.origin_x) / self.grid_size)
        grid_y_start = int((y - self.origin_y) / self.grid_size)
        grid_x_end = int((x + w - self.origin_x) / self.grid_size)
        grid_y_end = int((y + h - self.origin_y) / self.grid_size)

        # 确保在地图范围内
        grid_x_start = max(0, grid_x_start)
        grid_y_start = max(0, grid_y_start)
        grid_x_end = min(self.width - 1, grid_x_end)
        grid_y_end = min(self.height - 1, grid_y_end)

        # 标记障碍物
        for i in range(grid_y_start - 1, grid_y_end + 1):
            for j in range(grid_x_start - 1, grid_x_end + 1):
                self.grid_map[i][j] = 1

    def _mark_circle(self, x, y, r):
        """标记圆形障碍物"""
        # 将实际坐标转换为栅格坐标
        grid_x_center = int((x - self.origin_x) / self.grid_size)
        grid_y_center = int((y - self.origin_y) / self.grid_size)
        grid_radius = int(r / self.grid_size)

        # 标记障碍物 - 使用外接圆形方法
        for i in range(max(0, grid_y_center - grid_radius), min(self.height, grid_y_center + grid_radius)):
            for j in range(max(0, grid_x_center - grid_radius), min(self.width, grid_x_center + grid_radius)):
                # 计算栅格四个角点到圆心的距离
                corners = [
                    (self.origin_x + j * self.grid_size, self.origin_y + i * self.grid_size),  # 左下角
                    (self.origin_x + (j + 1) * self.grid_size, self.origin_y + i * self.grid_size),  # 右下角
                    (self.origin_x + j * self.grid_size, self.origin_y + (i + 1) * self.grid_size),  # 左上角
                    (self.origin_x + (j + 1) * self.grid_size, self.origin_y + (i + 1) * self.grid_size)  # 右上角
                ]

                # 如果任何一个角点在圆内，或者栅格与圆相交，则标记为障碍物
                for corner_x, corner_y in corners:
                    distance = ((corner_x - x) ** 2 + (corner_y - y) ** 2) ** 0.5
                    if distance < r:
                        self.grid_map[i][j] = 1
                        break

    def is_obstacle(self, x, y):
        """检查给定坐标是否为障碍物"""
        grid_x = x
        grid_y = y

        # 检查是否在地图范围内
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            return self.grid_map[grid_y][grid_x] == 1
        return True

    def get_grid_coordinates(self, x, y):
        """将实际坐标转换为栅格坐标"""
        grid_x = int((x - self.origin_x) / self.grid_size)
        grid_y = int((y - self.origin_y) / self.grid_size)
        return grid_x, grid_y

    def get_real_coordinates(self, grid_x, grid_y):
        """将栅格坐标转换为实际坐标（返回栅格中心点）"""
        x = self.origin_x + grid_x * self.grid_size + self.grid_size / 2
        y = self.origin_y + grid_y * self.grid_size + self.grid_size / 2
        return x, y

    def getGrid(self):
        return self.grid_map

    def getAllObstacles(self):
        res = []
        for grid_x in range(0, self.width):
            for grid_y in range(0, self.height):
                if self.grid_map[grid_y][grid_x] == 1:
                    res.append((grid_x, grid_y))
        return res

    def getObstacle(self, coord):
        x, y = self.get_grid_coordinates(coord.x, coord.y)
        return self.is_obstacle(x, y)

    def visualize(self, path=None, show_grid=True, save_path=None):
        """
        可视化栅格地图
        参数:
            path: 可选，路径点列表 [(x1,y1), (x2,y2), ...]
            show_grid: 是否显示栅格线
            save_path: 可选，保存图像的路径
        """
        # 创建图形
        fig, ax = plt.subplots(figsize=(10, 8))

        # 绘制栅格地图
        grid_array = np.array(self.grid_map)
        plt.imshow(grid_array, cmap='binary', origin='lower',
                   extent=[self.origin_x, self.origin_x + self.width * self.grid_size,
                           self.origin_y, self.origin_y + self.height * self.grid_size])

        # 绘制栅格线
        if show_grid:
            # 绘制垂直线
            for i in range(self.width + 1):
                x = self.origin_x + i * self.grid_size
                plt.plot([x, x], [self.origin_y, self.origin_y + self.height * self.grid_size], 'k-', alpha=0.2)
            # 绘制水平线
            for i in range(self.height + 1):
                y = self.origin_y + i * self.grid_size
                plt.plot([self.origin_x, self.origin_x + self.width * self.grid_size], [y, y], 'k-', alpha=0.2)

        # 绘制起点和终点
        start_x, start_y = self.env.start
        goal_x, goal_y = self.env.goal
        plt.plot(start_x, start_y, 'go', markersize=10, label='起点')
        plt.plot(goal_x, goal_y, 'ro', markersize=10, label='终点')

        # 绘制矩形障碍物轮廓
        for obs in self.env.obs_rectangle:
            x, y, w, h = obs
            rect = Rectangle((x, y), w, h, linewidth=1, edgecolor='r', facecolor='none')
            ax.add_patch(rect)

        # 绘制圆形障碍物轮廓
        for obs in self.env.obs_circle:
            x, y, r = obs
            circle = Circle((x, y), r, linewidth=1, edgecolor='r', facecolor='none')
            ax.add_patch(circle)

        # 绘制路径
        if path:
            path_x = [point[0] for point in path]
            path_y = [point[1] for point in path]
            plt.plot(path_x, path_y, 'b-', linewidth=2, label='路径')

        # 设置图形属性
        plt.title('栅格地图 (栅格大小: {})'.format(self.grid_size))
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')

        # 保存图像
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')

        plt.show()

# map = Env_Two()
# grid = Grid_Map(map)
# grid.visualize()
