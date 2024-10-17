import numpy as np
import matplotlib.pyplot as plt
import math
from mapData.map_one import mapOne
from mapData.map_two import mapTwo

# 定义机器人状态
class RobotState:
    def __init__(self, x, y, yaw, v, omega):
        self.x = x  # 机器人x坐标
        self.y = y  # 机器人y坐标
        self.yaw = yaw  # 机器人朝向角度（弧度）
        self.v = v  # 线速度
        self.omega = omega  # 角速度


# 定义DWA参数
class DWAParams:
    def __init__(self):
        self.max_speed = 0.4  # 最大速度
        self.min_speed = 0  # 最小速度（可倒退）
        self.max_yaw_rate = np.pi / 3.0  # 最大角速度
        self.max_accel = 0.2  # 最大加速度
        self.max_delta_yaw_rate = np.pi / 8.0  # 最大角加速度
        self.v_resolution = 0.01  # 速度分辨率
        self.yaw_rate_resolution = np.pi / 180.0  # 角速度分辨率
        self.dt = 0.1  # 时间间隔
        self.predict_time = 2.0  # 预测时间
        self.robot_radius = 0.15  # 机器人半径


# 障碍物类
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


# obstacles1 = [
#     Obstacle(2.22, 3.0, 0.085, -0, 0.4),
#     # Obstacle(1.2, 1.5, 0.02, 0.08, 0.25),
#     Obstacle(1.2, 1.5, 0.00, 0.00, 0.25),
#     # Obstacle(2, 2, 0.0, 0.0, 0.28),
#
#     Obstacle(4.0, 1.85, 0.0, 0.0, 0.4),
#     Obstacle(3.2, 3.5, 0.0, 0.072, 0.4),
#     Obstacle(4.2, 3, 0.0, 0, 0.4),
# ]

obstacles1 = [
    Obstacle(2.22, 3.0, 0.085, -0, 0.4),
    # Obstacle(1.2, 1.5, 0.02, 0.08, 0.25),
    Obstacle(1.2, 1.5, 0.00, 0.00, 0.25),
    # Obstacle(2, 2, 0.0, 0.0, 0.28),
    Obstacle(4.0, 1.85, 0.0, 0.0, 0.4),
    Obstacle(2.2, 3.9, 0.072, 0, 0.4),
    Obstacle(4.2, 3, 0.0, 0, 0.4),
]

alpha = 1
beta = 1
gamma = 1
delta = 0.9
theta = 0.2


# 机器人运动模型
def motion(state, v, omega, dt):
    state.x += v * np.cos(state.yaw) * dt
    state.y += v * np.sin(state.yaw) * dt
    state.yaw += omega * dt
    state.v = v
    state.omega = omega
    return state


# 计算动态窗口
def calc_dynamic_window(state, params):
    # 基于速度的动态窗口
    Vs = [params.min_speed, params.max_speed, -params.max_yaw_rate, params.max_yaw_rate]

    # 基于加速度限制的动态窗口
    Vd = [
        state.v - params.max_accel * params.dt,
        state.v + params.max_accel * params.dt,
        state.omega - params.max_delta_yaw_rate * params.dt,
        state.omega + params.max_delta_yaw_rate * params.dt
    ]

    # 动态窗口取交集
    dw = [
        max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
        max(Vs[2], Vd[2]), min(Vs[3], Vd[3])
    ]
    return dw


# 目标朝向代价函数
def __heading(trajectory, goal):
    dx = goal[0] - trajectory[-1][0]
    dy = goal[1] - trajectory[-1][1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1][2]
    cost = math.pi - abs(cost_angle)
    return cost


# 障碍物代价函数
def __dist(trajectory, obstacles, robot_radius):
    obstacle_cost = 10
    for obs in obstacles:
        for pos in trajectory:
            dist = np.linalg.norm(np.array([pos[0], pos[1]]) - np.array([obs.x, obs.y]))
            if dist <= robot_radius + obs.r:
                return float('inf')  # 碰撞，返回无限成本
            obstacle_cost = min(obstacle_cost, dist)
    return obstacle_cost


# 速度代价函数
def __vel(vel):
    return abs(vel)


# 转角代价函数
def __corn(state, trajectory, goal):
    m1 = math.atan2(state.x - goal[0], state.y - goal[1])
    m2 = math.atan2(trajectory[-1][0] - goal[0], trajectory[-1][1] - goal[1])
    cos_value = math.cos(abs(m1 - m2))
    return 1 + cos_value


# 动态障碍物代价函数
def __move(state, trajectory, obstacles, robot_radius):
    # 和动态障碍物的最小距离
    minDist = float('inf')
    # 计算动态障碍物和轨迹的角度差值
    minCost = 0
    move_cost = 0
    for obs in obstacles:
        if obs.vx != 0 or obs.vy != 0:
            cost = cosine_of_angle([obs.x, obs.y], [obs.x + obs.vx, obs.y + obs.vy], [state.x, state.y],
                                   [trajectory[- 1][0], trajectory[- 1][1]])
            lineDistance = np.linalg.norm(np.array([trajectory[- 1][0], trajectory[- 1][1]]) - np.array([obs.x, obs.y]))
            lineDistance = np.linalg.norm(np.array([state.x, state.y]) - np.array([obs.x, obs.y]))
            if cost is not None:
                # 进入安全距离
                if lineDistance < robot_radius * 3 + obs.r and lineDistance < minDist:
                    minDist = lineDistance
                    minCost = cost
    return minCost


# 安全距离内是否存在障碍物
def isSafe(trajectory, obstacles, robot_radius):
    flag = 0
    for pos in trajectory:
        for obs in obstacles:
            testDist = np.linalg.norm(np.array([pos[0], pos[1]]) - np.array([obs.x, obs.y]))
            if testDist < robot_radius * 2 + obs.r:
                flag = 1
                break
    return flag


# 评估轨迹，加入避障功能
def evaluate_trajectory(state, v, omega, params, goal, obstacles):
    predict_state = RobotState(state.x, state.y, state.yaw, v, omega)
    trajectory = [np.array([predict_state.x, predict_state.y, predict_state.yaw, predict_state.v, predict_state.omega])]
    time = 0.0

    while time <= params.predict_time:
        predict_state = motion(predict_state, v, omega, params.dt)
        trajectory.append(
            np.array([predict_state.x, predict_state.y, predict_state.yaw, predict_state.v, predict_state.omega]))
        time += params.dt

    trajectory = np.array(trajectory)
    return trajectory


# 计算夹角余弦值
def cosine_of_angle(A, B, C, D):
    # 计算向量AB和CD
    AB = np.array(B) - np.array(A)
    CD = np.array(D) - np.array(C)

    # 计算两个向量的点积
    dot_product = np.dot(AB, CD)

    # 计算向量的模
    norm_AB = np.linalg.norm(AB)
    norm_CD = np.linalg.norm(CD)

    if norm_AB == 0 or norm_CD == 0:
        return 0

    # 计算余弦值
    cosine_angle = dot_product / (norm_AB * norm_CD)

    return cosine_angle


# 判断动态窗口是否检测到动态障碍物
def isWindow(state, obstacles, trajectory, robot_radius):
    isDynamic = False
    for obs in obstacles:
        if obs.vx != 0 or obs.vy != 0:
            for pos in trajectory:
                lineDistance = np.linalg.norm(np.array([pos[0], pos[1]]) - np.array([obs.x, obs.y]))
                if lineDistance < robot_radius * 2 + obs.r:
                    isDynamic = True
                    break
    return isDynamic


# DWA主控制函数
def dwa_control(state, params, goal, obstacles):
    dw = calc_dynamic_window(state, params)
    min_cost = -float('inf')
    best_trajectory = None
    best_v = 0.0
    best_omega = 0.0

    sum_heading = 0
    sum_dist = 0
    sum_vel = 0

    sum_move = 0
    sum_corn = 0

    dynamicState = False
    safeState = False  # 是否进入静态状态

    for v in np.arange(dw[0], dw[1], params.v_resolution):
        for omega in np.arange(dw[2], dw[3], params.yaw_rate_resolution):
            trajectory = evaluate_trajectory(state, v, omega, params, goal, obstacles)
            heading_eval = __heading(trajectory, goal)
            dist_eval = __dist(trajectory, obstacles, params.robot_radius)

            vel_eval = __vel(trajectory[-1][3])
            sum_vel += vel_eval
            sum_dist += dist_eval
            sum_heading += heading_eval

            sum_corn += __corn(state, trajectory, goal)
            safeResult = isSafe(trajectory, obstacles, params.robot_radius)
            if safeResult:
                safeState = True
            if __move(state, trajectory, obstacles, params.robot_radius) != 0:
                sum_move += 1 - __move(state, trajectory, obstacles, params.robot_radius)
            # sum_move += 1 - __move(state, trajectory, obstacles, params.robot_radius)
            # 动态障碍物
            isDymanic = isWindow(state, obstacles, trajectory, params.robot_radius)
            if isDymanic:
                dynamicState = True

    isModify =1
    if isModify == 1:
        # 改进版本
        for v in np.arange(dw[0], dw[1], params.v_resolution):
            for omega in np.arange(dw[2], dw[3], params.yaw_rate_resolution):
                trajectory = evaluate_trajectory(state, v, omega, params, goal, obstacles)

                heading_eval = alpha * __heading(trajectory, goal) / sum_heading
                dist_eval = beta * __dist(trajectory, obstacles, params.robot_radius) / sum_dist
                vel_eval = gamma * __vel(trajectory[-1][3]) / sum_vel

                move_eval = __move(state, trajectory, obstacles, params.robot_radius)

                # 静态状况

                corn_vel = theta * __corn(state, trajectory, goal) / sum_corn

                # 存在动态障碍物的情况
                if dynamicState:
                    if abs(move_eval) < 0.4 and sum_move>0:
                        cost = heading_eval + dist_eval + vel_eval + delta * (1 - move_eval) / sum_move
                    elif move_eval < -0.7 and   sum_move>0:
                        cost = heading_eval + dist_eval + vel_eval + delta * (1 - move_eval) / sum_move
                    else:
                        cost = heading_eval + dist_eval + vel_eval
                elif safeState:
                    cost = heading_eval + dist_eval + vel_eval + corn_vel
                else:
                    cost = heading_eval + dist_eval + vel_eval

                if cost > min_cost:
                    min_cost = cost
                    best_trajectory = trajectory
                    best_v = v
                    best_omega = omega
    else:
        # 标准DWA
        for v in np.arange(dw[0], dw[1], params.v_resolution):
            for omega in np.arange(dw[2], dw[3], params.yaw_rate_resolution):
                trajectory = evaluate_trajectory(state, v, omega, params, goal, obstacles)

                heading_eval = alpha * __heading(trajectory, goal) / sum_heading
                dist_eval = beta * __dist(trajectory, obstacles, params.robot_radius) / sum_dist
                vel_eval = gamma * __vel(trajectory[-1][3]) / sum_vel
                cost = heading_eval + dist_eval + vel_eval

                if cost > min_cost:
                    min_cost = cost
                    best_trajectory = trajectory
                    best_v = v
                    best_omega = omega


    return best_v, best_omega, best_trajectory


# 可视化函数
def visualize(state, trajectory, start, goal, path, obstacles):
    plt.clf()
    plt.plot(start[0], start[1], marker='*', label="Start", markersize=12)  # 绘制目标点
    plt.plot(goal[0], goal[1], marker='*', label="Goal", markersize=12)  # 绘制目标点
    plt.plot(state.x, state.y, "xr", label="Robot")  # 绘制当前机器人位置
    plt.plot(np.array(path)[:, 0], np.array(path)[:, 1], "green", label="Path")  # 绘制路径
    if trajectory is not None:
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-g", label="Trajectory")  # 绘制预测轨迹
    for obs in obstacles:
        plt.plot(obs.x, obs.y, "ok", label="Obstacle", markersize=obs.r * 100)  # 绘制障碍物
    plt.grid(True)
    plt.xlim(0, 12)
    plt.ylim(0, 12)
    plt.axis("equal")
    plt.pause(0.001)


# 主程序
def main():
    # 初始状态
    mapData=mapTwo()
    obstacles1=mapData.getObstacles()
    state = RobotState(x=mapData.start[0], y=mapData.start[1], yaw=0.0, v=0.0, omega=0.0)
    start = np.array(mapData.start)
    goal = np.array(mapData.goal)
    params = DWAParams()
    path = []

    # 初始化动态障碍物
    obstacles = obstacles1
    total_path_length = 0.0  # 初始化路径长度

    plt.figure()
    index = 1
    if len(obstacles1) > 0:
        a = obstacles1[0].x
    while np.linalg.norm(np.array([state.x, state.y]) - goal) > 0.25:
        v, omega, trajectory = dwa_control(state, params, goal, obstacles)
        prev_x, prev_y = state.x, state.y
        state = motion(state, v, omega, params.dt)
        path.append([state.x, state.y])

        # 更新路径长度
        step_length = np.linalg.norm(np.array([state.x, state.y]) - np.array([prev_x, prev_y]))
        total_path_length += step_length

        visualize(state, trajectory, start, goal, path, obstacles)

        # 更新障碍物位置
        for obs in obstacles:
            obs.update(params.dt)
        index += 1

        # 计算运行时间
        # if len(obstacles1) > 0 and obstacles1[0].vx != 0:
        #     print(f"时间: {(obstacles1[0].x - a) / (obstacles1[0].vx)}")
    if len(obstacles1) > 0 and obstacles1[0].vx != 0:
        print(f"时间: {(obstacles1[0].x - a) / (obstacles1[0].vx)}")
    print(path)
    print(f'次数:{index}')
    print(f"Goal reached! Total path length: {total_path_length:.2f} units.")
    plt.show()


if __name__ == '__main__':
    main()
