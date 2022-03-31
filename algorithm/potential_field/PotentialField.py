import math
import time

import numpy as np

from data import obstacles
from data.Map import Map
from data.plotting import plot_clear, plot_set_title, plot_map, plot_path, plot_show, \
    plot_set_button_click_callback, plot_after_compute


class PotentialField:
    def __init__(self, env, start, goal, k_att, k_rep, rr, max_iter):
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
            self.MOTIONS[motion] = self.get_vector_angle(np.asarray(motion))
        # param
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

    @staticmethod
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

    def get_attractive_force(self):
        """
        计算引力大小
        :return:吸引力对应矢量
        """
        force = self.k_att * np.subtract(self.goal, self.current_pos)
        return force

    def get_repulsion_force(self):
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
                force = np.add(force, one_force)

        return force

    def get_total_force(self):
        """
        标准计算合力的方式
        :return:
        """
        return np.add(self.get_attractive_force(), self.get_repulsion_force())

    def get_next_pos(self, start, force):
        """
        计算下一个点位
        :return: 下一个点位的向量
        """
        force_angle = self.get_vector_angle(force)
        sub_dic = {}
        for motion in self.MOTIONS:
            angle_bias = abs(force_angle - self.MOTIONS[motion])
            if angle_bias < 45.0:
                sub_dic[motion] = angle_bias

        sub_dic = sorted(sub_dic.items(), key=lambda kv: kv[1])
        for dic in sub_dic:
            pos = np.add(start, np.asarray(dic[0]))
            if (pos[0], pos[1]) not in self.env.obs:
                return pos

        return np.add(start, np.asarray(sub_dic[0][0]))

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

    def detect_changes(self, event):
        x, y = event.xdata, event.ydata
        if x < 0 or x > self.env.x_range - 1 or y < 0 or y > self.env.y_range - 1:
            print(self.FLAG, "Please click right area!")
        else:
            node = (int(x), int(y))
            print(self.FLAG, "position:", node)

            if node not in self.env.obs:
                self.env.obs.add(node)
            else:
                self.env.obs.remove(node)

            self.path = [(self.start[0], self.start[1])]
            self.current_pos = self.start
            self.is_success = False
            self.iter = 0
            self.run_apf()
            plot_after_compute()

    def plan_path(self):
        while self.iter <= self.max_iter:
            if np.array_equal(self.current_pos, self.goal):
                self.is_success = True
                break
            self.iter += 1
            force = self.get_total_force()
            pos = self.get_next_pos(self.current_pos, force)
            # 触发局部最小值处理
            if (pos[0], pos[1]) in self.env.obs:
                break
            self.current_pos = pos
            self.path.append((self.current_pos[0], self.current_pos[1]))

    def run_apf(self):
        plot_clear()
        plot_set_title(self.FLAG)
        plot_map(self.env.obs)

        start_time = time.time()
        self.plan_path()
        end_time = time.time()
        if not self.is_success:
            print(self.FLAG, "Do not find path")
        else:
            print(self.FLAG, "path length:", self.get_path_len(), ", cost:", end_time - start_time, 's')
        plot_path(self.path, self.start, self.goal)

    def run(self):
        self.run_apf()
        print(self.FLAG, "start detect...")
        plot_set_button_click_callback(func=self.detect_changes)
        plot_show()


def main():
    # Error 存在陷入局部最小值问题
    # 主函数
    heuristic_type = "euclidean"
    env = Map(51, 31, heuristic_type=heuristic_type)
    obs, free = obstacles.get_anytime_standard_obs(env.x_range, env.y_range)
    env.update_obs(obs, free)

    # basic
    start = (5, 5)
    goal = (45, 25)
    k_att = 1.0
    k_rep = 0.8
    rr = 3
    max_iter = 1000
    demo = PotentialField(env, start, goal, k_att, k_rep, rr, max_iter)
    demo.run()


if __name__ == "__main__":
    main()
