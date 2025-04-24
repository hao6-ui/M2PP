class Env_Four:
    def __init__(self) -> object:
        self.start = [0.5, 0.5]
        self.goal = [9.5, 9]
        self.delta = 0
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle()
        self.name = '原图4'
        self.cost = 20
        self.gridSize = 0.02
    @staticmethod
    def obs_boundary():
        obs_boundary = [
            # [-11, -11, 1, 21],
            # [-11, 10, 21, 1],
            # [-10, -11, 21, 1],
            # [10, -10, 1, 21]
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
            [1, 1, 1, 1],
            [2, 4, 1, 1],
            [1, 6.5, 0.5, 1],
            [2, 9, 1, 0.5],
            [3, 2, 1, 1],
            [3.6, 8, 0.5, 1],
            [4, 1, 0.5, 0.5],
            [4, 6, 0.5, 1],
            [4.5, 3, 0.5, 2],
            [5.5, 2, 0.5, 1],
            [5.5, 5, 0.5, 2],
            [6, 1, 1, 0.5],
            [7, 4, 1, 1],
            [7, 6, 1, 1],
            [7, 8, 1, 1],
            [8.5, 0.5, 0.5, 1],
            [8.5, 2, 0.5, 1],
            [8.5, 6, 0.5, 1],
            [9.25, 7.5, 0.5, 1],
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [3, 6, 1],
            [6, 8, 1],
            [5, 2, 0.5],
            [9, 4, 1]
        ]
        return obs_cir
