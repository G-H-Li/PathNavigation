def init_free_set(x_range, y_range):
    x = x_range
    y = y_range
    free = set()
    for i in range(x):
        for j in range(y):
            if 0 < i < x-1 and 0 < j < y-1:
                free.add((i, j))

    return free


def get_flat_obs(x_range, y_range):
    # 21 21
    x = x_range
    y = y_range
    obs = set()
    free = init_free_set(x, y)

    obs_1 = {(4, 3), (4, 4), (5, 3), (5, 4)}
    obs_2 = {(7, 5), (7, 6)}
    obs_3 = {(5, 10), (6, 10)}
    obs_4 = {(10, 11), (10, 12), (11, 12)}
    obs_5 = {(15, 10), (16, 10)}
    obs_6 = {(13, 16), (13, 17)}
    obs_7 = {(18, 16), (19, 16)}

    obs.update(obs_1, obs_2, obs_3, obs_4, obs_5, obs_6, obs_7)
    free.difference_update(obs)
    return obs, free


def get_rough_obs(x_range, y_range):
    # 21 21
    x = x_range
    y = y_range
    obs = set()
    free = init_free_set(x, y)

    for i in range(4, 6):
        for j in range(3, 5):
            obs.add((i, j))

    for i in range(1, 4):
        for j in range(6, 8):
            obs.add((i, j))

    for i in range(3, 6):
        for j in range(11, 13):
            obs.add((i, j))

    for i in range(5, 9):
        for j in range(17, 19):
            obs.add((i, j))
    for i in range(5, 7):
        for j in range(15, 17):
            obs.add((i, j))

    for i in range(8, 11):
        obs.add((i, 11))
    for i in range(9, 12):
        obs.add((i, 12))
    for i in range(7, 11):
        obs.add((i, 10))

    for i in range(8, 13):
        for j in range(4, 6):
            obs.add((i, j))
    for i in range(9, 13):
        obs.add((i, 6))

    for i in range(12, 16):
        for j in range(19, 21):
            obs.add((i, j))
    for i in range(14, 16):
        for j in range(16, 19):
            obs.add((i, j))

    for i in range(15, 18):
        for j in range(10, 12):
            obs.add((i, j))
    for i in range(15, 17):
        obs.add((i, 9))

    for i in range(19, 21):
        for j in range(7, 10):
            obs.add((i, j))

    for i in range(16, 20):
        for j in range(3, 5):
            obs.add((i, j))

    free.difference_update(obs)
    return obs, free


def get_rug_obs(x_range, y_range):
    # 21 21
    x = x_range
    y = y_range
    obs = set()
    free = init_free_set(x, y)

    for i in range(2, 6):
        for j in range(18, 20):
            obs.add((i, j))

    for i in range(1, 5):
        for j in range(14, 17):
            obs.add((i, j))

    for i in range(2, 5):
        for j in range(11, 13):
            obs.add((i, j))

    for i in range(2, 4):
        for j in range(5, 8):
            obs.add((i, j))

    for i in range(2, 5):
        for j in range(2, 4):
            obs.add((i, j))

    for i in range(13, 16):
        for j in range(17, 19):
            obs.add((i, j))

    for i in range(7, 11):
        obs.add((i, 20))

    for i in range(14, 16):
        for j in range(1, 3):
            obs.add((i, j))

    for i in range(17, 20):
        for j in range(6, 9):
            obs.add((i, j))

    for j in range(6, 10):
        obs.add((6, j))
    for j in range(7, 9):
        obs.add((5, j))

    for i in range(7, 10):
        for j in range(13, 18):
            obs.add((i, j))
    for j in range(15, 17):
        obs.add((10, j))

    for i in range(9, 11):
        for j in range(8, 11):
            obs.add((i, j))
    for j in range(9, 11):
        obs.add((8, j))

    for i in range(8, 13):
        for j in range(4, 6):
            obs.add((i, j))
    for i in range(11, 13):
        for j in range(1, 3):
            obs.add((i, j))

    for j in range(8, 14):
        obs.add((12, j))
    for j in range(9, 15):
        obs.add((13, j))
    for j in range(10, 12):
        obs.add((14, j))

    for j in range(16, 20):
        obs.add((17, j))
    for j in range(15, 20):
        obs.add((18, j))
    for j in range(15, 17):
        obs.add((19, j))

    for j in range(10, 12):
        obs.add((16, j))
    for j in range(10, 13):
        obs.add((17, j))
    for j in range(11, 14):
        obs.add((18, j))

    for i in range(16, 20):
        obs.add((i, 4))
    for i in range(17, 20):
        obs.add((i, 3))
    for i in range(18, 19):
        obs.add((i, 2))

    free.difference_update(obs)
    return obs, free


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

