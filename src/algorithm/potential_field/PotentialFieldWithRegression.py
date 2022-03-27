import math

import numpy as np

from src.algorithm.potential_field.PotentialField import PotentialField
from src.data import obstacles
from src.data.Map import Map


class PotentialFieldWithRegression(PotentialField):
    def __init__(self, env, start, goal, k_att, k_rep, rr, max_iter, coefficient, dis_obs_goal, dis_goal_current):
        super().__init__(env, start, goal, k_att, k_rep, rr, max_iter)

        self.coefficient = coefficient
        self.dis_goal_current = dis_goal_current
        self.dis_obs_goal = dis_obs_goal
        self.FLAG = 'Potential Field with Regression '

    def get_attractive_force(self):
        """
        改进之后引力大小的计算
        :return: 引力对应矢量
        """
        dis_goal = math.hypot(self.current_pos[0] - self.goal[0], self.current_pos[1] - self.goal[1])
        force = self.k_att * np.subtract(self.goal, self.current_pos)
        if dis_goal <= self.coefficient:
            return force
        else:
            return self.coefficient / dis_goal * force

    def get_repulsion_force(self):
        return super().get_repulsion_force()

    def get_total_force(self):
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
            return self.get_attractive_force()
        else:
            return np.add(self.get_attractive_force(), self.get_repulsion_force())

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
        obs_set = set()
        if self.current_pos[1] - self.goal[1] == 0:
            x = self.current_pos[0]
            for y in range(self.env.y_range):
                node = (round(x), y)
                if node in self.env.obs:
                    obs_set.add(node)
        else:
            k = - (self.current_pos[0] - self.goal[0]) / (self.current_pos[1] - self.goal[1])
            b = self.current_pos[1] - k * self.current_pos[0]
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
        # TODO 获取障碍物边界，此处遇到问题，目前不知道下一步如何实现

    def plan_path(self):
        """
        改进之后的运行流程
        :return:
        """
        while self.iter <= self.max_iter:
            if np.array_equal(self.current_pos, self.goal):
                self.is_success = True
                break
            self.iter += 1
            force = self.get_total_force()
            pos = self.get_next_pos(force)
            # 触发局部最小值处理
            if (pos[0], pos[1]) in self.env.obs:
                self.deal_minima_problem()
            else:
                self.current_pos = pos
                self.path.append((self.current_pos[0], self.current_pos[1]))

        if self.is_success:
            self.optimize_path_by_regression()


def main():
    # basic
    start = (5, 5)
    goal = (45, 25)
    k_att = 1.0
    k_rep = 0.8
    rr = 3
    max_iter = 1000
    # 主函数
    heuristic_type = "euclidean"
    env = Map(51, 31, heuristic_type=heuristic_type)
    env.update_obs(obstacles.get_anytime_standard_obs(env.x_range, env.y_range))
    # improved
    coefficient = 30
    dis_goal_obs = 4
    dis_goal_current = 6
    demo = PotentialFieldWithRegression(env, start, goal, k_att, k_rep, rr, max_iter,
                                        coefficient, dis_goal_obs, dis_goal_current)
    demo.run()


if __name__ == "__main__":
    main()
