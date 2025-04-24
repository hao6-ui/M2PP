class Env_Two:
    def __init__(self) -> object:
        self.start = [-7.5, 5]
        self.goal = [5.5, 4]
        self.delta = 0
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle()
        self.name = '原图2'
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
            [-5, 3.5, 2, 2],
            [2, 3, 2,2]
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [0, 2, 1],
            [0, 3, 1],
            [0, 4, 1],
            [0, 5, 1],
            [0, 6, 1],
            [0, 7, 1],
            [0, 8, 1]
        ]
        return obs_cir
