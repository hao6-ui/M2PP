import copy
import statistics
import random
import math

from config.Plotting import Plotting
from config.Tree import TreeNode, Tree
from config.Utils import Utils

from config.Particle import Particle, Point
from mapData.map_two import Env_Two


class TPSO:
    def __init__(self, pointNum, popSize, maxIter, env, a, b):
        self.startPoint = env.start  # 起点
        self.goalPoint = env.goal  # 终点
        self.maxIter = maxIter  # 迭代次数
        self.popSize = popSize  # 种群大小
        self.pointNum = pointNum  # 控制点数量
        self.bestParticle = Particle()  # 全局最佳粒子

        self.obsCost = 2 * math.sqrt(
            pow((env.start[0] - env.goal[0]), 2) + pow((env.start[1] - env.goal[1]), 2))  # 障碍物代价

        self.w_inertial = 0.5
        self.w_social = 1
        self.w_cognitive = 0.6

        self.utils = Utils(env)
        self.plotting = Plotting(self.startPoint, self.goalPoint, env)
        self.env = env
        self.population = []  # 障碍物

        self.tabu_tree = ''  # 探索树

        self.a = 0.1  # 探索阶段阈值
        self.b = 0.5  # 加速阶段阈值

        self.line = 0.4  # 禁忌树父子区域之间选择几率

        self.bottom = min(self.startPoint[1], self.goalPoint[1])  # 地图上阈值
        self.top = max(self.startPoint[1], self.goalPoint[1])  # 地图下阈值

        for circle in env.obs_circle:
            self.bottom = min(self.bottom, circle[1] - circle[2])
            self.top = max(self.top, circle[1] + circle[2])

        for rectangle in env.obs_rectangle:
            self.bottom = min(self.bottom, rectangle[1] - rectangle[3])
            self.top = max(self.top, rectangle[1] + rectangle[3])
        self.top += 1
        self.bottom -= 1

        self.slope = (self.goalPoint[1] - self.startPoint[1]) / (self.goalPoint[0] - self.startPoint[0])  # 起终点斜率
        # 粒子群算法主流程

    def mainAlgorithm(self):

        # 初始化种群
        self.population = self.init_Particle()

        # 近五代最佳适应度
        pList = []

        # 迭代计算
        for i in range(self.maxIter):

            if len(pList) > 5:
                pList.pop(0)

            if len(pList) == 5:
                if statistics.stdev(pList) < self.a:
                    self.explore_search()
                    pList = [self.bestParticle.bestFitness]
                    print(f'探索阶段,最佳适应度{self.bestParticle.bestFitness}')
                elif statistics.stdev(pList) > self.b:
                    self.accelerate_search()
                    pList = [self.bestParticle.bestFitness]
                    print(f'加速阶段,最佳适应度{self.bestParticle.bestFitness}')
                else:
                    for particle in self.population:
                        self.optimize_Particle(particle)
            else:
                for particle in self.population:
                    self.optimize_Particle(particle)
            print(f'第{i + 1}次迭代,最佳适应度{self.bestParticle.bestFitness}')
            pList.append(self.bestParticle.bestFitness)
        return self.bestParticle

    # 初始化种群信息
    def init_Particle(self):
        stepSize = (self.goalPoint[0] - self.startPoint[0]) / (self.pointNum + 1.0)
        population = []
        for i in range(self.popSize):
            p = Particle()

            # 添加控制点
            p.position.append(self.startPoint)
            for j in range(1, self.pointNum + 1):
                randPoint = [self.startPoint[0] + j * stepSize,
                             random.uniform(self.bottom, self.top)]
                p.position.append(randPoint)
            p.position.append(self.goalPoint)

            p.velocity = [[0, 0] for _ in range(self.pointNum)]
            p.bestPos = copy.deepcopy(p.position)
            p.fitness = self.costFunction(p)
            p.bestFitness = p.fitness
            population.append(p)

            if self.bestParticle.bestFitness > p.fitness:
                self.bestParticle.bestFitness = p.fitness
                self.bestParticle.bestPos = copy.deepcopy(p.position)
        return population

    # 优化粒子
    def optimize_Particle(self, particle):
        self.updateParticleVelocity(particle)
        self.updateParticlePosition(particle)
        self.updateParticleFitness(particle)

    # 更新gbest,pbest
    def updateParticleFitness(self, particle):
        particle.fitness = self.costFunction(particle)
        if particle.fitness < particle.bestFitness:
            particle.bestFitness = particle.fitness
            particle.bestPos = copy.deepcopy(particle.position)
        if particle.bestFitness < self.bestParticle.bestFitness:
            self.bestParticle.bestFitness = particle.bestFitness
            self.bestParticle.bestPos = copy.deepcopy(particle.bestPos)

    # 计算种群的适应度函数，返回代价列表，用于后续选择操作
    def costFunction(self, particle):
        cost = 0
        for i in range(self.pointNum + 1):
            cost += self.pointCost(particle.position[i], particle.position[i + 1])
        return cost

    def pointCost(self, backPosition, prePosition):
        # 欧式距离
        distanceCost = math.sqrt(
            pow((backPosition[0] - prePosition[0]), 2) + pow((backPosition[1] - prePosition[1]), 2))
        # 碰撞物代价
        backPositionPoint = Point(backPosition)
        prePositionPoint = Point(prePosition)
        obstacleCost = self.utils.is_collision(backPositionPoint, prePositionPoint)
        if obstacleCost:
            distanceCost += 20
            # distanceCost += self.obsCost
        return distanceCost

    # 更新粒子位置
    def updateParticleVelocity(self, particle):
        for i in range(self.pointNum):
            rand1, rand2 = random.random(), random.random()
            vx, vy = particle.velocity[i]
            px, py = particle.position[i + 1]
            vx_new = self.w_inertial * vx + self.w_social * rand1 * (particle.bestPos[i + 1][0] - px) \
                     + self.w_cognitive * rand2 * (self.bestParticle.bestPos[i + 1][0] - px)
            vy_new = self.w_inertial * vy + self.w_social * rand1 * (particle.bestPos[i + 1][1] - py) \
                     + self.w_cognitive * rand2 * (self.bestParticle.bestPos[i + 1][1] - py)
            particle.velocity[i] = [vx_new, vy_new]

    # 更新粒子位置
    def updateParticlePosition(self, particle):
        for i in range(self.pointNum):
            px = particle.position[i + 1][0] + particle.velocity[i][0]
            py = particle.position[i + 1][1] + particle.velocity[i][1]
            px = self.startPoint[0] if px < self.startPoint[0] else px
            px = self.goalPoint[0] if px > self.goalPoint[0] else px
            py = self.bottom if py < self.bottom else py
            py = self.top if py > self.top else py

            particle.position[i + 1] = [round(px, 2), round(py, 2)]
        particle.position.sort(key=lambda p: p[0])

    # 加速阶段
    def accelerate_search(self):
        for particle in self.population:
            self.updateParticlePosition(particle)
            if self.accelerate_cost(particle) < self.accelerate_cost(particle):
                particle.bestPos = copy.deepcopy(particle.position)
                particle.bestFitness = self.accelerate_fitness(particle)

    # 计算pbest位置代价
    def accelerate_fitness(self, particle):
        cost = 0
        for i in range(self.pointNum + 1):
            cost += self.pointCost(particle.bestPos[i], particle.bestPos[i + 1])
        return cost

    # 加速阶段新代价函数
    def accelerate_cost(self, particle):
        cost = 0
        obstacle_cost = 1000
        for i in range(self.pointNum + 1):
            backPosition, prePosition = particle.position[i], particle.position[i + 1]
            pointCost = self.pointCost(backPosition, prePosition) + self.slope_cost(backPosition, prePosition)
            cost += pointCost
            obstacle_cost = min(obstacle_cost, self.obstacle_recent_cost(backPosition, prePosition))
        if obstacle_cost != 1000:
            cost += obstacle_cost
        return cost

    # 斜率代价计算
    def slope_cost(self, backPosition, prePosition):
        if backPosition[0] == prePosition[0]:
            return 0
        else:
            slope = (backPosition[1] - prePosition[1]) / (backPosition[0] - prePosition[0])
            return abs(slope - self.slope)

    # 障碍物贴近计算
    def obstacle_recent_cost(self, backPosition, prePosition):
        if backPosition[0] == prePosition[0]:
            return 0
        # 碰撞不考虑
        if self.pointCost(backPosition, prePosition) == 20:
            return 0
        A, B, C = self.line_from_points(backPosition, prePosition)
        if A == 0 and B == 0 and C == 0:
            return 0
        cost = 1000
        # 计算最近障碍物距离
        for circle in self.env.obs_circle:
            cost = min(cost, self.point_to_line_distance(circle, A, B, C))
        for rectangle in self.env.obs_rectangle:
            x, y = (rectangle[0] + rectangle[2]) / 2, (rectangle[1] + rectangle[3]) / 2
            cost = min(cost, self.point_to_line_distance([x, y], A, B, C))
        if cost != 1000:
            return cost
        return 0

    # 计算两点之间的直线
    def line_from_points(self, backPosition, prePosition):
        x1, x2, y1, y2 = backPosition[0], prePosition[0], backPosition[1], prePosition[1],
        # 计算 A, B, C 的值
        A = y2 - y1
        B = x1 - x2
        if B == 0:
            return 0, 0, 0
        C = A * x1 + B * y1
        # 返回直线方程 Ax + By + C = 0 中的 A, B, C
        return A, B, -C

    # 计算点到直线的距离
    def point_to_line_distance(self, point, A, B, C):
        x1, y1 = point[0], point[1]
        # 计算点到直线的距离
        distance = abs(A * x1 + B * y1 + C) / math.sqrt(A ** 2 + B ** 2)
        return distance

    # 探索阶段
    def explore_search(self):
        for particle in self.population:
            for i in range(1, self.pointNum + 1):
                newx, newy = self.generatePosition_Y()
                newx = self.xLine(particle.position[i - 1][0], particle.position[i + 1][0], newx)
                tmpData_A = copy.deepcopy(particle.position[i - 1])
                tmpData_B = copy.deepcopy(particle.position[i + 1])
                self.generateTree([particle.position[i], [newx, newy], tmpData_A, tmpData_B])
                preCost = self.pointCost(particle.position[i - 1], particle.position[i]) + self.pointCost(
                    particle.position[i + 1], particle.position[i])
                nowCost = self.pointCost(particle.position[i - 1], [newx, newy]) + self.pointCost(
                    [newx, newy], particle.position[i])
                if nowCost <= preCost:
                    particle.position[i][0], particle.position[i][1] = newx, newy
                self.updateParticleFitness(particle)
        # 重置探索树
        self.tabu_tree = ''

    def xLine(self, left, right, x):
        if x < left:
            return left
        if x > right:
            return right
        return x

    def generatePosition(self, position):
        left, right, bottom, top = position[0], position[1], position[2], position[3]
        for p in position:
            left = p if p[0] < left[0] else left
            right = p if p[0] > right[0] else right
            bottom = p if p[1] < bottom[1] else bottom
            top = p if p[1] > top[1] else top
        return [left, right, bottom, top]

    # 生成禁忌树
    def generateTree(self, position):
        position = self.generatePosition(position)
        nowNode = TreeNode(position)
        if self.tabu_tree == '':
            self.tabu_tree = Tree(nowNode)
        else:
            self.tabu_tree.addNode(nowNode, self.tabu_tree.root)

    # 根据禁忌树生成X轴解
    def generatePosition_X(self, left, right):
        ans = random.uniform(left, right)
        return ans

    # 随机获取高度
    def randomHigh(self, high):
        sumHigh = (1 + high) * high // 2
        odds = random.randint(1, sumHigh)
        for i in range(1, high):
            if odds <= i:
                return high - i + 1
        return 1

    # 根据禁忌树生成Y轴解
    def generatePosition_Y(self):
        tree = self.tabu_tree
        # 无探索树的情况
        if tree == '':
            top, bottom = self.top, self.bottom
            left, right = 1000, -1000
            for particle in self.population:
                for i in range(1, len(particle.position) - 1):
                    position = particle.position[i]
                    top = max(position[1], top)
                    bottom = min(position[1], bottom)
                    left = min(left, position[0])
                    right = max(right, position[0])
        else:
            # 获取层数

            level = self.randomHigh(tree.high)

            self.tabu_tree.getNode(tree.root, level, 0)

            node = self.tabu_tree.node

            # 区域选择概率
            odds = random.random()

            if level == 1:
                top, bottom = max(node[0][1], node[1][1]), min(node[0][1], node[1][1])
                if odds <= self.line:
                    bottom = top
                    top = self.top
                elif odds <= self.line * 2:
                    top = bottom
                    bottom = self.bottom
            else:
                top, bottom = max(node[0][1], node[1][1]), min(node[0][1], node[1][1])
                if odds <= self.line:
                    bottom = top
                    top = max(node[2][1], node[3][1])
                elif odds <= self.line * 2:
                    top = bottom
                    bottom = min(node[2][1], node[3][1])
            left, right = min(node[0][0], node[1][0]), max(node[0][0], node[1][0])
        # return random.uniform(bottom, top)
        return random.uniform(left, right), random.uniform(bottom, top)


if __name__ == '__main__':
    env = Env_Two()
    PSOAlgorithm = TPSO(8, 100, 100, env, 0.1, 1)
    traj = PSOAlgorithm.mainAlgorithm().bestPos
    print(traj)
    path = []
    for i in range(PSOAlgorithm.pointNum + 2):
        path.append(Point(traj[i]))

    PSOAlgorithm.plotting.animation([], path, '', [], 'TPSO', animation=True)
