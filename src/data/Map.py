import matplotlib.pyplot as plt


class Map:
    """
    地图类，控制整个地图范围
    """
    def __init__(self, x_range=51, y_range=31, obs=None, start=(5, 5), goal=(45, 25)):
        if obs is None:
            obs = set()
        self.x_range = x_range
        self.y_range = y_range
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.start = start
        self.goal = goal
        self.obs = obs

    def update_obs(self, obs):
        self.obs = obs

    def draw_map(self):
        """
        绘制地图
        :return:
        """
        obs_x = [x[0] for x in self.obs]
        obs_y = [x[1] for x in self.obs]

        plt.plot(self.start[0], self.start[1], "bs")
        plt.plot(self.goal[0], self.goal[1], "gs")
        plt.plot(obs_x, obs_y, "ks")
        plt.axis("equal")
        plt.show()
