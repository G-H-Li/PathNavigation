import matplotlib.pyplot as plt

from src.data import obstacles


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
        """
        更新障碍物集合
        :param obs: 集合
        :return: None
        """
        self.obs = obs

    def draw_map(self):
        """
        绘制地图
        :return: None
        """
        obs_x = [x[0] for x in self.obs]
        obs_y = [x[1] for x in self.obs]

        plt.plot(self.start[0], self.start[1], "bs")
        plt.plot(self.goal[0], self.goal[1], "gs")
        plt.plot(obs_x, obs_y, "ks")
        plt.axis("equal")


def main():
    _map = Map()
    _map.update_obs(obstacles.get_static_obs(_map.x_range, _map.y_range))
    _map.draw_map()
    plt.show()


if __name__ == "__main__":
    main()
