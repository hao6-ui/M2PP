class Env_Two:
    def __init__(self) -> object:
        self.start = [-10, -9]
        self.goal = [9, 10]
        self.delta = 0
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle()

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
            [-7.5, -8, 1, 8],
            [-1.5, -1.5, 3, 3],
            [3, -5, 6, 4]
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [-5, 6, 3],
            [-2, -6, 2],
            [6, 6, 2],
            [-4, 1.6, 1.2]
        ]
        return obs_cir
