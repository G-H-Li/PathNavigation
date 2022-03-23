def get_anytime_error_obs(x_range, y_range):
    x = x_range
    y = y_range
    obs = set(get_grid_obs(x, y))

    for i in range(2, 21):
        obs.add((i, 7))
    for i in range(2, 21):
        obs.add((i, 3))
    return obs


def get_anytime_standard_obs(x_range, y_range):
    x = x_range
    y = y_range
    obs = set(get_grid_obs(x, y))

    for i in range(10, 21):
        obs.add((i, 15))
    for i in range(15):
        obs.add((20, i))

    for i in range(15, 30):
        obs.add((30, i))
    for i in range(16):
        obs.add((40, i))

    return obs


def get_static_obs(x_range, y_range):
    """
    根据x, y范围，设置静态障碍物位置
    :param x_range: x轴位置上限
    :param y_range: y轴位置上限
    :return: 障碍物集合
    """
    x = x_range
    y = y_range
    obs = set(get_grid_obs(x, y))

    for i in range(x // 5, 2 * x // 5 + 1):
        obs.add((i, y // 2))
    for i in range(y // 2):
        obs.add((2 * x // 5, i))

    for i in range(2 * y // 5, y):
        obs.add((3 * x // 5, i))
    for i in range(y // 2 + 1):
        obs.add((4 * x // 5, i))

    return obs


def get_grid_obs(x_range, y_range):
    x = x_range
    y = y_range
    obs = set()

    for i in range(x):
        obs.add((i, 0))
    for i in range(x):
        obs.add((i, y - 1))

    for i in range(y):
        obs.add((0, i))
    for i in range(y):
        obs.add((x - 1, i))

    return obs

