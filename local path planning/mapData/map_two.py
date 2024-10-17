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


class mapTwo:

    def __init__(self) -> object:
        self.start = [30.25, 20.25]
        self.goal = [33.7, 22.5]


    def getObstacles(self):
        return [
            Obstacle(30.5, 22, 0.001, 0, 0.1),
            Obstacle(30.7, 21, 0, 0, 0.15),
            Obstacle(31.5, 20.5, 0, 0, 0.2),
            Obstacle(31.5, 21.3, 0, 0, 0.2),
            # Obstacle(32.7, 21.5, 0, 0, 0.15),
            Obstacle(33.5, 21.7, 0, 0, 0.32),
        ]


