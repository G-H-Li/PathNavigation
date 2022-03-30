import math

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

from src.data import obstacles
from src.data.Map import Map


class FuzzyLogic:
    def __init__(self, env, start, goal, rr):
        self.env = env
        self.start = start
        self.goal = goal
        self.direction = None
        self.path = [self.start]
        self.rr = rr
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
        self.obs_dis['near'] = fuzz.gaussmf(self.obs_dis.universe, 0, 1.3)
        self.obs_dis['far'] = fuzz.gaussmf(self.obs_dis.universe, self.rr, 1.3)
        # if is_show:
        #     self.obs_dis.view()

        self.obs_angle['left'] = fuzz.gaussmf(self.obs_angle.universe, -90, 75)
        self.obs_angle['right'] = fuzz.gaussmf(self.obs_angle.universe, 90, 75)
        # if is_show:
        #     self.obs_angle.view()

        self.goal_dis['near'] = fuzz.gaussmf(self.goal_dis.universe, 0, self.dis_start_end*7/16)
        self.goal_dis['far'] = fuzz.gaussmf(self.goal_dis.universe, self.dis_start_end, self.dis_start_end*7/16)
        if is_show:
            self.goal_dis.view()

        self.goal_angle['left'] = fuzz.gaussmf(self.obs_angle.universe, -90, 75)
        self.goal_angle['right'] = fuzz.gaussmf(self.obs_angle.universe, 90, 75)
        if is_show:
            self.goal_angle.view()

        self.run_angle['left'] = fuzz.gaussmf(self.obs_angle.universe, -180, 150)
        self.run_angle['right'] = fuzz.gaussmf(self.obs_angle.universe, 180, 150)
        self.run_angle['targetDirection'] = fuzz.gaussmf(self.obs_angle.universe, 0, 8)
        if is_show:
            self.run_angle.view()

    def run(self):
        self.generate_membership_func(True)


def main():
    heuristic_type = "euclidean"
    env = Map(51, 31, heuristic_type=heuristic_type)
    env.update_obs(obstacles.get_potential_field_obs(env.x_range, env.y_range))

    # basic
    start = (5, 5)
    goal = (45, 25)
    rr = 3
    demo = FuzzyLogic(env, start, goal, rr)
    demo.run()


if __name__ == "__main__":
    main()
