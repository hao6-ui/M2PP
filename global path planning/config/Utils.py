import math
import numpy as np
from config.Particle import Point


class Utils:
    def __init__(self, env):
        self.env = env
        self.delta = self.env.delta
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def get_delta(self):
        return self.delta

    def update_obs(self, obs_cir, obs_bound, obs_rec):
        self.obs_circle = obs_cir
        self.obs_boundary = obs_bound
        self.obs_rectangle = obs_rec

    def get_obs_vertex(self):
        delta = self.delta
        obs_list = []

        for (ox, oy, w, h) in self.obs_rectangle:
            vertex_list = [[ox - delta, oy - delta],
                           [ox + w + delta, oy - delta],
                           [ox + w + delta, oy + h + delta],
                           [ox - delta, oy + h + delta]]
            obs_list.append(vertex_list)

        return obs_list

    def is_intersect_rec(self, start, end, o, d, a, b):
        v1 = [o[0] - a[0], o[1] - a[1]]
        v2 = [b[0] - a[0], b[1] - a[1]]
        v3 = [-d[1], d[0]]

        div = np.dot(v2, v3)

        if div == 0:
            return False

        t1 = np.linalg.norm(np.cross(v2, v1)) / div
        t2 = np.dot(v1, v3) / div

        if t1 >= 0 and 0 <= t2 <= 1:
            shot = Point((o[0] + t1 * d[0], o[1] + t1 * d[1]))
            dist_obs = self.get_dist(start, shot)
            dist_seg = self.get_dist(start, end)
            if dist_obs <= dist_seg:
                return True

        return False

    def is_intersect_circle(self, o, d, a, r):
        d2 = np.dot(d, d)
        delta = self.delta

        if d2 == 0:
            return False

        t = np.dot([a[0] - o[0], a[1] - o[1]], d) / d2

        if 0 <= t <= 1:
            shot = Point((o[0] + t * d[0], o[1] + t * d[1]))
            if self.get_dist(shot, Point(a)) <= r + delta:
                return True

        return False

    def is_collision(self, start, end):
        if self.is_inside_obs(start) or self.is_inside_obs(end):
            return True

        o, d = self.get_ray(start, end)
        obs_vertex = self.get_obs_vertex()

        for (v1, v2, v3, v4) in obs_vertex:
            if self.is_intersect_rec(start, end, o, d, v1, v2):
                return True
            if self.is_intersect_rec(start, end, o, d, v2, v3):
                return True
            if self.is_intersect_rec(start, end, o, d, v3, v4):
                return True
            if self.is_intersect_rec(start, end, o, d, v4, v1):
                return True

        for (x, y, r) in self.obs_circle:
            if self.is_intersect_circle(o, d, [x, y], r):
                return True

        return False

    def is_collision_new(self, start, end, grid):

        if grid.getObstacle(start) == True or grid.getObstacle(end) == True:
            return True

        x1, y1 = start.x, start.y
        x2, y2 = end.x, end.y

        # minX = min(x1, x2)
        # maxX = max(x1, x2)
        # minY = min(y1, y2)
        # maxY = max(y1, y2)
        #
        # grid_x1, grid_y1 = grid.get_grid_coordinates(minX, minY)
        # grid_x2, grid_y2 = grid.get_grid_coordinates(maxX, maxY)

        # for x in range(grid_x1, grid_x2 + 1):
        #     for y in range(grid_y1, grid_y2 + 1):
        #         if grid.is_obstacle(x, y) == True:
        #             return True
        x1, y1 = grid.get_grid_coordinates(x1, y1)
        x2, y2 = grid.get_grid_coordinates(x2, y2)

        # print(x1, y1)

        dx = x2 - x1
        dy = y2 - y1
        x, y = x1, y1
        points = []  # 存储经过的格子
        if abs(dx) >= abs(dy):  # 水平步长优先
            sx = 1 if dx > 0 else -1
            sy = dy / abs(dx) if dx != 0 else 0
            for i in range(abs(dx) + 1):
                if grid.is_obstacle(int(x), int(y)) == True:
                    return True
                points.append((int(x), int(y)))  # 添加坐标
                x += sx
                y += sy
        else:  # 垂直步长优先
            sy = 1 if dy > 0 else -1
            sx = dx / abs(dy) if dy != 0 else 0
            for i in range(abs(dy) + 1):
                if grid.is_obstacle(int(x), int(y)) == True:
                    return True
                points.append((int(x), int(y)))  # 添加坐标
                x += sx
                y += sy
        # print(points)
        # return points
        return False

        x0, x1 = grid_x1, grid_x2
        y0, y1 = grid_y1, grid_y2

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
            if grid.is_obstacle(x0, y0) == True:
                return True

        return False

    def is_inside_obs(self, Point):
        delta = self.delta

        for (x, y, r) in self.obs_circle:
            if math.hypot(Point.x - x, Point.y - y) <= r + delta:
                return True

        for (x, y, w, h) in self.obs_rectangle:
            if 0 <= Point.x - (x - delta) <= w + 2 * delta \
                    and 0 <= Point.y - (y - delta) <= h + 2 * delta:
                return True

        for (x, y, w, h) in self.obs_boundary:
            if 0 <= Point.x - (x - delta) <= w + 2 * delta \
                    and 0 <= Point.y - (y - delta) <= h + 2 * delta:
                return True

        return False

    @staticmethod
    def get_ray(start, end):
        orig = [start.x, start.y]
        direc = [end.x - start.x, end.y - start.y]
        return orig, direc

    # 计算两点之间直线距离
    @staticmethod
    def get_dist(start, end):
        return math.hypot(end.x - start.x, end.y - start.y)

    def Kinematic_check(self, nearestPoint, newRealPoint, goalPoint):
        angleOne = math.degrees(math.atan2(newRealPoint.y - nearestPoint.y, newRealPoint.x - nearestPoint.x))
        angleTwo = math.degrees(math.atan2(goalPoint.y - nearestPoint.y, goalPoint.x - nearestPoint.x))
        # print(abs(angleTwo - angleOne))
        if abs(angleTwo - angleOne) > 90:
            return True
        return False
