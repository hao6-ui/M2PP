class Particle:
    def __init__(self) -> None:
        self.reset()

    def reset(self):
        self.position = []
        self.velocity = []
        self.fitness = -1
        self.bestPos = []
        self.bestFitness = float('inf')

        # EEPSO特有参数
        self.s=0
        self.f=0

class Point:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None