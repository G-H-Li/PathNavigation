import math
import time

import matplotlib.pyplot as plt
import numpy as np

from src.data import obstacles
from src.data.Map import Map
from src.data.plotting import plot_clear, plot_set_title, plot_map, plot_path, plot_show


def get_vector_angle(vector):
    """
    根据合力大小，计算向量与水平角度
    :param vector: 向量
    :return:角度
    """
    simple = np.asarray((1, 0))
    degree = np.arccos(simple.dot(vector) / (np.linalg.norm(simple) * np.linalg.norm(vector))) * 180 / np.pi
    if vector[1] < 0.0:
        degree = 360 - degree
    return degree


class PotentialField:
    def __init__(self, env, start, goal, k_att, k_rep, rr, max_iter, coefficient, dis_obs_goal, dis_goal_current):
        """
        构造类
        :param env: 环境
        :param start: 起点
        :param goal: 终点
        :param k_att: 引力系数
        :param k_rep: 斥力系数
        :param rr: 斥力作用范围
        :param max_iter: 最大迭代次数
        """
        # map info
        self.goal = np.asarray(goal, dtype=float)
        self.env = env
        self.start = np.asarray(start, dtype=float)
        self.current_pos = np.asarray(start, dtype=float)
        self.MOTIONS = {}
        for motion in self.env.motions:
            self.MOTIONS[motion] = get_vector_angle(np.asarray(motion))
        # param
        self.coefficient = coefficient
        self.dis_goal_current = dis_goal_current
        self.dis_obs_goal = dis_obs_goal
        self.rr = rr
        self.k_rep = k_rep
        self.k_att = k_att
        # 算法迭代
        self.iter = 0
        self.max_iter = max_iter
        # result
        self.path = [start]
        self.is_success = False
        # info
        self.FLAG = "Artificial Potential Field:"

    def get_attractive_force_standard(self):
        """
        计算引力大小
        :return:吸引力对应矢量
        """
        force = self.k_att * np.subtract(self.goal, self.current_pos)
        return force

    def get_attractive_force_improved(self):
        """
        改进之后引力大小的计算
        :return: 引力对应矢量
        """
        dis_goal = math.hypot(self.current_pos[0] - self.goal[0], self.current_pos[1] - self.goal[1])
        force = self.get_attractive_force_standard()
        if dis_goal <= self.coefficient:
            return force
        else:
            return self.coefficient / dis_goal * force

    def get_repulsion_force_standard(self):
        """
        计算斥力大小之和
        :return:所有斥力之和矢量
        """
        force = np.zeros(2)
        for obs in self.env.obs:
            # 计算obs与当前点距离
            dis = math.hypot(self.current_pos[0] - obs[0], self.current_pos[1] - obs[1])
            if dis <= self.rr:
                one_force = np.subtract(self.current_pos, np.asarray(obs)) * self.k_rep \
                            * (1 / dis - 1 / self.rr) \
                            / (dis ** 2)
                np.add(force, one_force)

        return force

    def get_repulsion_force_improved(self):
        """
        改进之后的斥力计算
        :return: 斥力之和矢量
        """
        return self.get_repulsion_force_standard()

    def get_total_force_standard(self):
        """
        标准计算合力的方式
        :return:
        """
        return np.add(self.get_attractive_force_standard(), self.get_repulsion_force_standard())

    def get_total_force_improved(self):
        """
        改进计算合力方式
        :return:
        """
        min_dis = float('inf')
        min_obs = 0
        for obs in self.env.obs:
            if math.hypot(self.current_pos[0] - obs[0], self.current_pos[1] - obs[1]) < min_dis:
                min_obs = obs
        dis_qc = math.hypot(min_obs[0] - self.goal[0],
                            min_obs[1] - self.goal[1])
        dis_gr = math.hypot(self.current_pos[0] - self.goal[0], self.current_pos[1] - self.goal[1])
        if dis_qc <= self.dis_obs_goal and dis_gr <= self.dis_goal_current:
            return self.get_attractive_force_improved()
        else:
            return np.add(self.get_attractive_force_improved(), self.get_repulsion_force_improved())

    def get_next_pos(self, force):
        """
        计算下一个点位
        :return: 下一个点位的向量
        """
        force_angle = get_vector_angle(force)
        sub_dic = {}
        for motion in self.MOTIONS:
            sub_dic[motion] = abs(force_angle - self.MOTIONS[motion])

        sub_dic = sorted(sub_dic.items(), key=lambda kv: kv[1])
        return np.add(self.current_pos, np.asarray(sub_dic[0][0]))

    def get_path_len(self):
        """
        计算path的长度
        :return: 路径长度
        """
        length = 0
        for idx, node in enumerate(self.path):
            if idx > 0:
                length += math.hypot(node[0] - self.path[idx - 1][0], node[1] - self.path[idx - 1][1])

        return length

    def optimize_path_by_regression(self):
        """
        基于回归的方法进行路径优化
        :return:
        """
        pass

    def deal_minima_problem(self):
        """
        处理局部最小值问题
        :return:
        """
        k = - (self.current_pos[0] - self.goal[0]) / (self.current_pos[1] - self.goal[1])
        b = self.current_pos[1] - k * self.current_pos[0]
        obs_set = set()
        bias = abs(k / 2)
        # 找到路径上障碍物
        for x in range(self.env.x_range):
            nodes = set()
            nodes.add((x, round(k * x + b - bias)))
            nodes.add((x, round(k * x + b + bias)))
            nodes.add((x, math.ceil(k * x + b)))
            nodes.add((x, math.floor(k * x + b)))
            for node in nodes:
                if node in self.env.obs:
                    obs_set.add(node)
                    break
        # 寻找距离当前位置最近的两个障碍物
        print(len(obs_set), obs_set)

    def run_standard(self):
        plot_clear()
        plot_set_title(self.FLAG)
        plot_map(self.env.obs)

        start_time = time.time()

        while self.iter <= self.max_iter:
            if np.array_equal(self.current_pos, self.goal):
                self.is_success = True
                break
            self.iter += 1
            force = self.get_total_force_standard()
            pos = self.get_next_pos(force)
            # 触发局部最小值处理
            if (pos[0], pos[1]) in self.env.obs:
                break
            self.current_pos = pos
            self.path.append((self.current_pos[0], self.current_pos[1]))

        if not self.is_success:
            print(self.FLAG, "Do not find path")

        end_time = time.time()
        print(self.FLAG, "path length:", self.get_path_len(), ", cost:", end_time - start_time, 's')
        plot_path(self.path, self.start, self.goal)
        plot_show()

    def run_improved(self):
        """
        改进之后的运行流程
        :return:
        """
        plot_clear()
        plot_set_title(self.FLAG)
        plot_map(self.env.obs)

        start_time = time.time()

        while self.iter <= self.max_iter:
            if np.array_equal(self.current_pos, self.goal):
                self.is_success = True
                break
            self.iter += 1
            force = self.get_total_force_improved()
            pos = self.get_next_pos(force)
            # 触发局部最小值处理
            if (pos[0], pos[1]) in self.env.obs:
                self.deal_minima_problem()
            else:
                self.current_pos = pos
                self.path.append((self.current_pos[0], self.current_pos[1]))

        if not self.is_success:
            print(self.FLAG, "Do not find path")
        else:
            self.optimize_path_by_regression()

        end_time = time.time()
        print(self.FLAG, "path length:", self.get_path_len(), ", cost:", end_time - start_time, 's')
        plot_path(self.path, self.start, self.goal)
        plot_show()


def main():
    # 主函数
    heuristic_type = "euclidean"
    env = Map(51, 31, heuristic_type=heuristic_type)
    env.update_obs(obstacles.get_anytime_standard_obs(env.x_range, env.y_range))

    # basic
    start = (5, 5)
    goal = (45, 25)
    k_att = 1.0
    k_rep = 100.0
    rr = 10
    max_iter = 1000
    # improved
    coefficient = 30
    dis_goal_obs = 4
    dis_goal_current = 6
    demo = PotentialField(env, start, goal, k_att, k_rep, rr, max_iter, coefficient, dis_goal_obs, dis_goal_current)
    demo.run_improved()


if __name__ == "__main__":
    main()
