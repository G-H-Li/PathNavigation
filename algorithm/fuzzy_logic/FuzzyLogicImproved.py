import skfuzzy as fuzz

from algorithm.fuzzy_logic.FuzzyLogic import FuzzyLogic
from data import obstacles
from data.Map import Map


class FuzzyLogicImproved(FuzzyLogic):
    def __init__(self, env, start, goal, rr, show_param):
        super().__init__(env, start, goal, rr, show_param)

    def generate_rules(self):
        pass

    def generate_membership_func(self, is_show):
        self.obs_dis['near'] = fuzz.smf(self.obs_dis.universe, 0, self.rr/2)
        self.obs_dis['far'] = fuzz.trimf(self.obs_dis.universe, [0, self.rr/2, self.rr])
        if is_show:
            self.obs_dis.view()

        self.goal_dis['near'] = fuzz.zmf(self.goal_dis.universe, 0, self.dis_start_end * 7 / 16)
        self.goal_dis['far'] = fuzz.piecemf(self.goal_dis.universe, [0, self.dis_start_end/2, self.dis_start_end])
        if is_show:
            self.goal_dis.view()


def main():
    heuristic_type = "euclidean"
    env = Map(51, 31, heuristic_type=heuristic_type)
    env.update_obs(obstacles.get_grid_obs(env.x_range, env.y_range))

    # basic
    start = (5, 5)
    goal = (45, 25)
    rr = 3
    demo = FuzzyLogicImproved(env, start, goal, rr, True)
    demo.run()


if __name__ == "__main__":
    main()
