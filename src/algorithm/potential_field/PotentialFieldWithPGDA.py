import math

import numpy as np

from src.algorithm.potential_field.PotentialField import PotentialField
from src.data import obstacles
from src.data.Map import Map


class PotentialFieldWithPGDA(PotentialField):
    def __init__(self, env, start, goal, k_att, k_rep, rr, max_iter, radius_goal):
        super().__init__(env, start, goal, k_att, k_rep, rr, max_iter)
        # 距离目标的半径距离
        self.radius_goal = radius_goal
        self.is_minimal = 0
        self.FLAG = "Potential Field with GDA:"

    def get_attractive_force(self):
        dis = math.hypot(self.current_pos[0] - self.goal[0], self.current_pos[1] - self.goal[1])
        force = self.k_att * np.subtract(self.goal, self.current_pos)
        if dis < self.radius_goal:
            return force
        else:
            return force / dis

    def get_repulsion_force(self):
        force = np.zeros(2)
        for obs in self.env.obs:
            # 计算obs与当前点距离
            dis = math.hypot(self.current_pos[0] - obs[0], self.current_pos[1] - obs[1])
            if dis <= self.rr:
                one_force = np.subtract(self.current_pos, np.asarray(obs)) * self.k_rep \
                            * (1 / dis - 1 / self.rr) \
                            / (dis ** 3)
                force = np.add(force, one_force)

        return force

    def get_total_force(self):
        dis = math.hypot(self.current_pos[0] - self.goal[0], self.current_pos[1] - self.goal[1])
        if dis < self.radius_goal:
            total_force = self.get_attractive_force()
        else:
            total_force = np.add(self.get_attractive_force(), self.get_repulsion_force())

        if self.is_minimal > 0:
            total_force = np.add(total_force, self.get_repulsion_force() * self.is_minimal)

        return total_force

    def plan_path(self):
        # TODO 困在急弯中，且存在震荡问题
        while self.iter < self.max_iter:
            if np.array_equal(self.current_pos, self.goal):
                self.is_success = True
                break
            self.iter += 1
            force = self.get_total_force()
            pos = self.get_next_pos(self.current_pos, force)
            # 触发局部最小值处理
            if (pos[0], pos[1]) in self.env.obs:
                self.is_minimal += 1
                continue
            else:
                self.is_minimal = 0
                self.current_pos = pos
                self.path.append((self.current_pos[0], self.current_pos[1]))


def main():
    # basic
    start = (5, 5)
    goal = (45, 25)
    k_att = 1.0
    k_rep = 0.8
    rr = 3
    max_iter = 1000
    radius_goal = 15
    # 主函数
    heuristic_type = "euclidean"
    env = Map(51, 31, heuristic_type=heuristic_type)
    env.update_obs(obstacles.get_anytime_standard_obs(env.x_range, env.y_range))

    demo = PotentialFieldWithPGDA(env, start, goal, k_att, k_rep, rr, max_iter, radius_goal)
    demo.run()


if __name__ == "__main__":
    main()
