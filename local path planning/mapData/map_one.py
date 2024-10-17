class Obstacle:
    def __init__(self, x, y, vx, vy, r):
        self.x = x  # 障碍物的x坐标
        self.y = y  # 障碍物的y坐标
        self.vx = vx  # 障碍物的x方向速度
        self.vy = vy  # 障碍物的y方向速度
        self.r = r

    # 更新障碍物位置
    def update(self, dt):
        self.x += self.vx * dt
        self.y += self.vy * dt


class mapOne:

    def __init__(self) -> object:
        self.start = [1.0, 1.0]
        self.goal = [4.0, 5.0]


    def getObstacles(self):
        return [
            Obstacle(2.22, 3.0, 0.085, -0, 0.4),
            # Obstacle(1.2, 1.5, 0.02, 0.08, 0.25),
            Obstacle(1.2, 1.5, 0.00, 0.00, 0.25),
            # Obstacle(2, 2, 0.0, 0.0, 0.28),
            Obstacle(4.0, 1.85, 0.0, 0.0, 0.4),
            Obstacle(2.2, 3.9, 0.072, 0, 0.4),
            Obstacle(4.2, 3, 0.0, 0, 0.4),
        ]
