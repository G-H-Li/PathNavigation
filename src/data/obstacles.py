
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

    for i in range(x):
        obs.add((i, 0))
    for i in range(x):
        obs.add((i, y - 1))

    for i in range(y):
        obs.add((0, i))
    for i in range(y):
        obs.add((x - 1, i))

    for i in range(x // 5, 2 * x // 5 + 1):
        obs.add((i, y // 2))
    for i in range(y // 2):
        obs.add((2 * x // 5, i))

    for i in range(1 * y // 3, y):
        obs.add((3 * x // 5, i))
    for i in range(y // 2 + 1):
        obs.add((4 * x // 5, i))

    return obs

