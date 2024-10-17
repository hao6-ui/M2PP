class Env_One:
    def __init__(self) -> object:
        self.start = [-9, 2.5]
        self.goal = [10, 0.5]
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
            [-8, 2, 4, 1],
            [-4, -1, 4, 4],
            [7, -2, 1.5, 2]
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [2, 4, 1],
            [8.5, 2.5, 1]
        ]
        return obs_cir
