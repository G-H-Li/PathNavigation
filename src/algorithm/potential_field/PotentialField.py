import numpy as np

from src.data.Map import Map


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
        # param
        self.rr = rr
        self.k_rep = k_rep
        self.k_att = k_att
        # 算法迭代
        self.iter = 0
        self.max_iter = max_iter
        # result
        self.path = []
        self.error = False

    def get_attractive_force(self):
        """
        计算引力大小
        :return:
        """
        force = self.k_att * np.subtract(self.goal, self.current_pos)
        return force

    def get_repulsion_force(self):
        """
        计算斥力大小之和
        :return:
        """
        pass

    def run(self):
        pass


def main():
    # 主函数
    start = (5, 5)
    goal = (45, 25)
    k_att = 1.0
    k_rep = 100.0
    rr = 3
    max_iter = 500
    demo = PotentialField(Map(), start, goal, k_att, k_rep, rr, max_iter)
    print(demo.get_attractive_force())


if __name__ == "__main__":
    main()
