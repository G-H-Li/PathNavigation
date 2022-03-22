import matplotlib.pyplot as plt


def plot_map(obs):
    """
    地图的绘制方法
    :param obs: 地图
    :return:
    """
    # 绘制障碍物
    obs_x = [x[0] for x in obs]
    obs_y = [x[1] for x in obs]

    plt.plot(obs_x, obs_y, "ks")
    plt.axis("equal")


def plot_visited(visited, color='gray'):
    for node in visited:
        plt.plot(node[0], node[1], marker='s', color=color)


def plot_path(path, start, goal, color='r'):
    path_x = [path[i][0] for i in range(len(path))]
    path_y = [path[i][1] for i in range(len(path))]
    plt.plot(path_x, path_y, linewidth='3', color=color)

    plt.plot(start[0], start[1], 'b^')
    plt.plot(goal[0], goal[1], 'g^')

    plt.pause(0.1)


def plot_show():
    plt.show()


def plot_set_title(title):
    plt.title(title)

