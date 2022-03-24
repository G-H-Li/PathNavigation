import math
import time
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
            self.MOTIONS[motion] = get_vector_angle(np.asarray(motion))
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

    def get_attractive_force_standard(self):
        """
        计算引力大小
        :return:吸引力对应矢量
        """
        force = self.k_att * np.subtract(self.goal, self.current_pos)
        return force

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
        for dic in sub_dic:
            tmp = np.add(self.current_pos, np.asarray(dic[0]))
            if (tmp[0], tmp[1]) in self.env.obs:
                continue
            else:
                return tmp
        return None

    def get_path_len(self):
        """
        计算path的长度
        :return: 路径长度
        """
        length = 0
        for idx, node in enumerate(self.path):
            if idx > 0:
                length += math.hypot(node[0] - self.path[idx-1][0], node[1] - self.path[idx-1][1])

        return length

    def optimize_path_by_regression(self):
        """
        基于回归的方法进行路径优化
        :return:
        """
        pass

    def run(self):
        plot_clear()
        plot_set_title(self.FLAG)
        plot_map(self.env.obs)

        start_time = time.time()

        while self.iter <= self.max_iter:
            if np.array_equal(self.current_pos, self.goal):
                self.is_success = True
                break
            self.iter += 1
            force = np.add(self.get_attractive_force_standard(), self.get_repulsion_force_standard())
            self.current_pos = self.get_next_pos(force)
            if self.current_pos is None:
                break
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
    env.update_obs(obstacles.get_grid_obs(env.x_range, env.y_range))

    start = (5, 5)
    goal = (45, 25)
    k_att = 1.0
    k_rep = 100.0
    rr = 10
    max_iter = 1000
    demo = PotentialField(env, start, goal, k_att, k_rep, rr, max_iter)
    demo.run()


if __name__ == "__main__":
    main()
