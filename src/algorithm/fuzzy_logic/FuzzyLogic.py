import math
import time

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

from src.data import obstacles
from src.data.Map import Map
from src.data.plotting import plot_map, plot_set_title, plot_clear, plot_path, plot_show


class FuzzyLogic:
    def __init__(self, env, start, goal, rr, show_param=False):
        self.env = env
        self.start = start
        self.goal = goal
        self.current_pos = start
        self.path = [self.start]
        self.rr = rr
        self.rules = []
        self.system = None
        self.MOTIONS = {}
        for motion in self.env.motions:
            self.MOTIONS[motion] = self.get_vector_angle(np.asarray(motion))
        self.show_param_fig = show_param
        self.is_success = True
        self.FLAG = "Fuzzy Logic:"

        self.dis_start_end = int(math.hypot(start[0] - goal[0], start[1] - goal[1]))
        obs_dis_range = np.arange(0, self.rr, 0.1)
        angle_range = np.arange(-180, 180, 1)
        goal_dis_range = np.arange(0, self.dis_start_end, 0.5)

        # input
        self.obs_dis = ctrl.Antecedent(obs_dis_range, "obsDistance")
        self.obs_angle = ctrl.Antecedent(angle_range, "obsAngle")
        self.goal_dis = ctrl.Antecedent(goal_dis_range, "goalDistance")
        self.goal_angle = ctrl.Antecedent(angle_range, "goalAngle")

        # output
        self.run_angle = ctrl.Consequent(angle_range, "runAngle")

    def generate_membership_func(self, is_show):
        self.obs_dis['near'] = fuzz.gaussmf(self.obs_dis.universe, 0, self.rr*7/16)
        self.obs_dis['far'] = fuzz.gaussmf(self.obs_dis.universe, self.rr, self.rr*7/16)
        if is_show:
            self.obs_dis.view()

        self.obs_angle['left'] = fuzz.gaussmf(self.obs_angle.universe, -180, 150)
        self.obs_angle['right'] = fuzz.gaussmf(self.obs_angle.universe, 180, 150)
        if is_show:
            self.obs_angle.view()

        self.goal_dis['near'] = fuzz.gaussmf(self.goal_dis.universe, 0, self.dis_start_end*7/16)
        self.goal_dis['far'] = fuzz.gaussmf(self.goal_dis.universe, self.dis_start_end, self.dis_start_end*7/16)
        if is_show:
            self.goal_dis.view()

        self.goal_angle['left'] = fuzz.gaussmf(self.obs_angle.universe, -180, 150)
        self.goal_angle['right'] = fuzz.gaussmf(self.obs_angle.universe, 180, 150)
        self.goal_angle['targetDirection'] = fuzz.gaussmf(self.obs_angle.universe, 0, 8)
        if is_show:
            self.goal_angle.view()

        self.run_angle['turn-left'] = fuzz.gaussmf(self.obs_angle.universe, -180, 150)
        self.run_angle['turn-right'] = fuzz.gaussmf(self.obs_angle.universe, 180, 150)
        self.run_angle['zero'] = fuzz.gaussmf(self.obs_angle.universe, 0, 8)
        if is_show:
            self.run_angle.view()

        self.run_angle.defuzzify_method = 'centroid'

    def generate_rules(self):
        rule1 = ctrl.Rule(antecedent=self.goal_angle['right'], consequent=self.run_angle['turn-right'])
        rule2 = ctrl.Rule(antecedent=self.goal_angle['left'], consequent=self.run_angle['turn-left'])
        rule3 = ctrl.Rule(antecedent=(self.obs_dis['near'] & self.goal_dis['far'] & self.obs_angle['left']),
                          consequent=self.run_angle['turn-right'])
        rule4 = ctrl.Rule(antecedent=(self.obs_dis['near'] & self.goal_dis['far'] & self.obs_angle['right']),
                          consequent=self.run_angle['turn-left'])
        rule5 = ctrl.Rule(antecedent=self.goal_angle['targetDirection'], consequent=self.run_angle['zero'])
        rule6 = ctrl.Rule(antecedent=(self.obs_dis['far'] & self.goal_angle['targetDirection']),
                          consequent=self.run_angle['zero'])
        self.rules = [rule1, rule2, rule3, rule4, rule5, rule6]

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
            degree = -degree
        return degree

    def get_min_obs_under_rr(self):
        """
        获取rr范围内距离最近的障碍物
        :return: 障碍物的位置，和与当前点距离
        """
        min_dis_pos = [float('inf'), float('inf')]
        min_dis = float('inf')
        for x in range(int(self.current_pos[0]-self.rr), int(self.current_pos[0]+self.rr+1)):
            for y in range(int(self.current_pos[1]-self.rr), int(self.current_pos[1]+self.rr+1)):
                dis = math.hypot(x-self.current_pos[0], y-self.current_pos[1])
                if dis < self.rr and (x, y) in self.env.obs and dis < min_dis:
                    min_dis_pos = (x, y)
                    min_dis = dis

        return min_dis_pos, min_dis

    def get_next_pos(self, angle):
        """
        计算下一个点位
        :return: 下一个点位的向量
        """
        sub_dic = {}
        for motion in self.MOTIONS:
            angle_bias = abs(angle - self.MOTIONS[motion])
            if angle_bias < 45.0:
                sub_dic[motion] = angle_bias

        sub_dic = sorted(sub_dic.items(), key=lambda kv: kv[1])
        for dic in sub_dic:
            pos = np.add(np.asarray(self.current_pos), np.asarray(dic[0]))
            if (pos[0], pos[1]) not in self.env.obs:
                return pos

        return np.add(np.asarray(self.current_pos), np.asarray(sub_dic[0][0]))

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

    def explore_path(self):
        self.generate_membership_func(is_show=self.show_param_fig)
        self.generate_rules()
        self.system = ctrl.ControlSystemSimulation(ctrl.ControlSystem(rules=self.rules))

        while self.current_pos != self.goal:
            min_obs, min_obs_dis = self.get_min_obs_under_rr()
            obs_angle = self.get_vector_angle(np.subtract(np.asarray(min_obs), np.asarray(self.current_pos)))
            goal_angle = self.get_vector_angle(np.subtract(np.asarray(self.goal), np.asarray(self.current_pos)))
            goal_dis = math.hypot(self.current_pos[0] - self.goal[0], self.current_pos[1] - self.goal[1])

            self.system.input['obsDistance'] = min_obs_dis
            self.system.input['obsAngle'] = obs_angle
            self.system.input['goalDistance'] = goal_dis
            self.system.input['goalAngle'] = goal_angle
            self.system.compute()
            run_angle = self.system.output['runAngle']
            pos = self.get_next_pos(run_angle)
            if (pos[0], pos[1]) in self.env.obs:
                self.is_success = False
                break
            else:
                self.current_pos = (pos[0], pos[1])
                self.path.append(self.current_pos)

    def run(self):
        plot_clear()
        plot_set_title(self.FLAG)
        plot_map(self.env.obs)

        start_time = time.time()
        self.explore_path()
        end_time = time.time()
        if not self.is_success:
            print(self.FLAG, "Do not find path")
        else:
            print(self.FLAG, "path length:", self.get_path_len(), ", cost:", end_time - start_time, 's')
        plot_path(self.path, self.start, self.goal)
        plot_show()


def main():
    heuristic_type = "euclidean"
    env = Map(51, 31, heuristic_type=heuristic_type)
    env.update_obs(obstacles.get_anytime_standard_obs(env.x_range, env.y_range))

    # basic
    start = (5, 5)
    goal = (45, 25)
    rr = 3
    demo = FuzzyLogic(env, start, goal, rr, False)
    demo.run()


if __name__ == "__main__":
    main()
