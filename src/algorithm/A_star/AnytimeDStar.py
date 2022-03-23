import math
import time

from src.data import obstacles
from src.data.Map import Map
from src.data.plotting import plot_set_title, plot_map, plot_visited, plot_path, plot_set_button_click_callback, \
    plot_show, plot_clear, plot_after_compute


class AnytimeDStar:
    def __init__(self, env, start, goal, eps, heuristic_type):
        """
        构造函数
        :param env: 算法运行的地图 Map
        :param eps: 膨胀因子
        :param heuristic_type: 启发类型，计算距离的方法
        """
        # algorithm info
        self.heuristic_type = heuristic_type
        self.eps = eps
        self.goal = goal
        self.start = start
        # map info
        self.env = env
        # store structure
        self.g, self.rhs, self.OPEN, self.INCONS = {}, {}, {}, {}
        self.CLOSED = set()
        # initiation
        for i in range(env.x_range):
            for j in range(env.y_range):
                self.g[(i, j)] = float('inf')
                self.rhs[(i, j)] = float('inf')
        self.rhs[self.start] = float('inf')
        self.rhs[self.goal] = 0.0
        self.OPEN[self.goal] = self.get_key(self.goal)
        # dynamic
        self.obs_add = set()
        self.obs_remove = set()
        # display
        self.visited = set()
        self.FLAG = 'Anytime D Star:'
        # result
        self.path = []
        self.error = 0

    def get_h(self, start, goal):
        """
        根据起始点、结束点、启发方法来计算两点之间距离
        :param goal: 结束点
        :param start: 起始点
        :return:
        """
        heuristic_type = self.heuristic_type  # heuristic type

        if heuristic_type == "manhattan":
            # 曼哈顿距离
            return abs(goal[0] - start[0]) + abs(goal[1] - start[1])
        elif heuristic_type == "euclidean":
            # 欧几里得距离
            return math.hypot(goal[0] - start[0], goal[1] - start[1])

    def get_key(self, s):
        """
        获取输入点的key值
        :param s: 数组 点
        :return: [start至当前点的代价, 当前点到goal的代价]
        """
        if self.g[s] > self.rhs[s]:
            return [self.rhs[s] + self.eps * self.get_h(self.start, s), self.rhs[s]]
        else:
            return [self.g[s] + self.get_h(self.start, s), self.g[s]]

    def is_reachable(self, start, goal):
        """
        判断此点motion可达的点是否是障碍物
        :param start: 起始点
        :param goal: 结束点
        :return: boolean
        """
        if start[0] != goal[0] and start[1] != goal[1]:
            return True

        heuristic_type = self.heuristic_type
        if start in self.env.obs or goal in self.env.obs:
            return False
        else:
            if heuristic_type == "manhattan":
                return True
            elif heuristic_type == "euclidean":
                # 欧几里得距离需要判断两边是否是障碍物
                if start[0] - goal[0] == goal[1] - start[1]:
                    s1 = (min(start[0], goal[0]), min(start[1], goal[1]))
                    s2 = (max(start[0], goal[0]), max(start[1], goal[1]))
                else:
                    s1 = (min(start[0], goal[0]), max(start[1], goal[1]))
                    s2 = (max(start[0], goal[0]), min(start[1], goal[1]))

                if s1 in self.env.obs and s2 in self.env.obs:
                    return False
        return True

    def get_cost(self, start, goal):
        """
        根据起止点计算代价，一般是临近点
        :param start:
        :param goal:
        :return:
        """
        # 判断路径上有无障碍物
        if not self.is_reachable(start, goal):
            return float('inf')
        # 正常返回cost
        else:
            return self.get_h(start, goal)

    def get_neighbor(self, s):
        """
        返回当前点可访问的临近点
        :param s: node
        :return: 集合
        """
        neighbors = set()
        for u in self.env.motions:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.env.obs:
                neighbors.add(s_next)
        return neighbors

    def get_mini_key(self):
        """
        获取OPEN字典中的最key值对
        :return:key ， value
        """
        s = min(self.OPEN, key=self.OPEN.get)
        return s, self.OPEN[s]

    def update_state(self, s):
        """
        更新节点状态
        :param s: node
        :return:
        """
        if s != self.goal:
            for node in self.get_neighbor(s):
                self.rhs[s] = min(self.rhs[s], self.g[node] + self.get_cost(s, node))

        if s in self.OPEN:
            self.OPEN.pop(s)

        if self.g[s] != self.rhs[s]:
            if s not in self.CLOSED:
                self.OPEN[s] = self.get_key(s)
            else:
                self.INCONS[s] = 0

    def find_mini_next_node(self, s):
        g_list = {}
        for node in self.get_neighbor(s):
            if self.is_reachable(s, node):
                g_list[node] = self.g[node]
        return min(g_list, key=g_list.get)

    def compute_or_improve_path(self):
        """
        计算或者改进路径
        :return:
        """
        while True:
            key, value = self.get_mini_key()
            if value >= self.get_key(self.start) and self.rhs[self.start] == self.g[self.start]:
                break

            self.OPEN.pop(key)
            self.visited.add(key)

            if self.g[key] > self.rhs[key]:
                self.g[key] = self.rhs[key]
                self.CLOSED.add(key)
                for node in self.get_neighbor(key):
                    self.update_state(node)
            else:
                self.g[key] = float('inf')
                for node in self.get_neighbor(key):
                    self.update_state(node)
                self.update_state(key)

    def publish_path(self):
        """
        正向搜索查找路径
        :return: 路径列表
        """
        s = self.start
        self.path = [s]

        while True:
            if s == self.goal:
                break
            if len(self.path) > self.env.x_range * self.env.y_range:
                print(self.FLAG, "Do not find path")
                self.error += 1
                break
            s = self.find_mini_next_node(s)
            self.path.append(s)

    def explore_path(self):
        plot_clear()
        plot_set_title("Anytime D *: e=" + str(self.eps))
        plot_map(self.env.obs)
        self.visited = set()

        start_time = time.time()
        self.compute_or_improve_path()
        end_time = time.time()

        print(self.FLAG, "e:", self.eps, ', path length:', self.g[self.start], ', cost:', end_time-start_time, 's')
        plot_visited(self.visited)
        self.publish_path()
        plot_path(self.path, self.start, self.goal)

    def optimize_path(self):
        while True:
            if self.eps <= 1.0:
                break
            self.eps -= 0.5
            self.OPEN.update(self.INCONS)
            for s in self.OPEN:
                self.OPEN[s] = self.get_key(s)
            self.CLOSED = set()
            self.explore_path()

    def detect_changes(self, event):
        """
        此处检测点击事件，并执行路径规划
        小变动与大变动的区别与应用场景有关，此处根据环境变化次数进行区分
        :param event: 点击事件
        :return:
        """
        x, y = event.xdata, event.ydata
        if x < 0 or x > self.env.x_range - 1 or y < 0 or y > self.env.y_range - 1:
            print(self.FLAG, "Please click right area!")
        else:
            node = (int(x), int(y))
            print(self.FLAG, "position:", node)

            if node not in self.env.obs:
                self.obs_add.add(node)
                if node in self.obs_remove:
                    self.obs_remove.remove(node)

                if node == self.start:
                    self.error += 1
                    self.start = self.find_mini_next_node(self.start)
                if node == self.goal:
                    self.error += 1
                    self.goal = self.find_mini_next_node(self.goal)
                self.env.obs.add(node)
                self.g[node] = float('inf')
                self.rhs[node] = float('inf')
            else:
                self.obs_remove.add(node)
                if node in self.obs_add:
                    self.obs_add.remove(node)

                self.env.obs.remove(node)
                self.update_state(node)

            for sn in self.get_neighbor(node):
                self.update_state(sn)

            if self.error >= 1:  # significant changes
                self.error = 0
                self.eps += 1.5

                for s in self.obs_add:
                    for sn in self.get_neighbor(s):
                        self.update_state(sn)

                for s in self.obs_remove:
                    for sn in self.get_neighbor(s):
                        self.update_state(sn)
                    self.update_state(s)

                self.optimize_path()
            else:  # small changes
                while True:
                    if len(self.INCONS) == 0:
                        break
                    self.OPEN.update(self.INCONS)
                    for s in self.OPEN:
                        self.OPEN[s] = self.get_key(s)
                    self.CLOSED = set()
                    self.explore_path()

                    if self.eps <= 1.0:
                        break

            plot_after_compute()

    def run(self):
        """
        运行主方法
        :return:
        """
        start_time = time.time()

        self.explore_path()
        self.optimize_path()

        end_time = time.time()
        print(self.FLAG, "total cost:", end_time-start_time, 's')
        # 当eps=1时，检测地图变化
        print(self.FLAG, "start detect...")
        plot_set_button_click_callback(func=self.detect_changes)
        plot_show()


def main():
    """
    main func
    TODO anytime D star 障碍物更新时，容易陷入无路径的情况，具体原因未知
    :return:
    """
    heuristic_type = "euclidean"

    env = Map(51, 31, heuristic_type=heuristic_type)
    env.update_obs(obstacles.get_anytime_error_obs(env.x_range, env.y_range))

    start = (5, 5)
    goal = (5, 25)
    demo = AnytimeDStar(env, start, goal, 2.0, heuristic_type)
    demo.run()


if __name__ == '__main__':
    main()
