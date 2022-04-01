def init_free_set(x_range, y_range):
    x = x_range
    y = y_range
    free = set()
    for i in range(x):
        for j in range(y):
            if 0 < i < x-1 and 0 < j < y-1:
                free.add((i, j))

    return free


def get_ga_obs(x_range, y_range):
    x = x_range
    y = y_range
    obs = set()
    free = init_free_set(x, y)

    for i in range(3, 6):
        for j in range(2, 5):
            obs.add((i, j))

    for i in range(8, 16):
        for j in range(4, 6):
            obs.add((i, j))

    for i in range(1, 9):
        for j in range(13, 15):
            obs.add((i, j))

    for i in range(8, 12):
        obs.add((15, i))

    for i in range(10, 13):
        for j in range(12, 15):
            obs.add((i, j))

    free.difference_update(obs)
    return obs, free


def get_anytime_standard_obs(x_range, y_range):
    x = x_range
    y = y_range
    obs = set()
    free = init_free_set(x, y)

    for i in range(10, 21):
        obs.add((i, 15))
    for i in range(15):
        obs.add((20, i))

    for i in range(15, 30):
        obs.add((30, i))
    for i in range(16):
        obs.add((40, i))

    free.difference_update(obs)
    return obs, free


def get_static_obs(x_range, y_range):
    """
    根据x, y范围，设置静态障碍物位置
    :param x_range: x轴位置上限
    :param y_range: y轴位置上限
    :return: 障碍物集合
    """
    x = x_range
    y = y_range
    obs = set()
    free = init_free_set(x, y)

    for i in range(x // 5, 2 * x // 5 + 1):
        obs.add((i, y // 2))
    for i in range(y // 2):
        obs.add((2 * x // 5, i))

    for i in range(2 * y // 5, y):
        obs.add((3 * x // 5, i))
    for i in range(y // 2 + 1):
        obs.add((4 * x // 5, i))

    free.difference_update(obs)
    return obs, free


def get_grid_obs(x_range, y_range):
    x = x_range
    y = y_range
    obs = set()
    free = init_free_set(x, y)

    for i in range(x):
        obs.add((i, 0))
    for i in range(x):
        obs.add((i, y - 1))

    for i in range(y):
        obs.add((0, i))
    for i in range(y):
        obs.add((x - 1, i))

    return obs, free

