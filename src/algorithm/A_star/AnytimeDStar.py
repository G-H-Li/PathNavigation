from src.data import obstacles
from src.data.Map import Map


class AnytimeDStar:
    def __init__(self):
        """

        """

    def run(self):
        """

        :return:
        """


def main():
    """
    main func
    :return:
    """
    _map = Map()
    _map.update_obs(obstacles.get_static_obs(_map.x_range, _map.y_range))
    _map.draw_map()


if __name__ == '__main__':
    main()
