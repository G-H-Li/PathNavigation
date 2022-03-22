import math

from src.data import obstacles
from src.data.Map import Map


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
        self.obs = env.obs
        self.u_set = env.motions  # next input set
        self.x_range = env.x_range
        self.y_range = env.y_range
        # store structure
        self.g, self.rhs, self.OPEN, self.INCONS = {}, {}, {}, {}
        self.CLOSED = set()
        # initiation
        for i in range(env.x_range):
            for j in range(env.y_range):
                self.g[(i, j)] = float('inf')
        self.rhs[self.start] = float('inf')
        self.rhs[self.goal] = 0.0
        self.OPEN[self.goal] = self.get_key(self.goal)

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
        if start in self.obs or goal in self.obs:
            return False
        else:
            if heuristic_type == "manhattan":
                return True
            elif heuristic_type == "euclidean":
                # 欧几里得距离需要判断两边是否是障碍物
                if abs(start[0] - goal[0]) == abs(start[1] - goal[1]):
                    s1 = (min(start[0], goal[0]), min(start[1], goal[1]))
                    s2 = (max(start[0], goal[0]), max(start[1], goal[1]))
                else:
                    s1 = (min(start[0], goal[0]), max(start[1], goal[1]))
                    s2 = (max(start[0], goal[0]), min(start[1], goal[1]))

                if s1 in self.obs and s2 in self.obs:  # TODO 此处or与and区别有待测试
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
        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.obs:
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
            self.rhs[s] = float('inf')  # TODO 此步是否必要
            for node in self.get_neighbor(s):
                self.rhs[s] = min(self.rhs[s], self.g[node] + self.get_cost(s, node))

        if s in self.OPEN:
            self.OPEN.pop(s)

        if self.g[s] != self.rhs[s]:
            if s not in self.CLOSED:
                self.OPEN[s] = self.get_key(s)
            else:
                self.INCONS[s] = 0

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
        path = [s]

        while True:
            if s == self.goal:
                break
            if len(path) > self.x_range * self.y_range:
                print("Do not find path")
                break

            g_list = {}
            for node in self.get_neighbor(s):
                if self.is_reachable(s, node):
                    g_list[node] = self.g[node]
            s = min(g_list, key=g_list.get)
            path.append(s)

        return path

    def detect_changes(self):
        pass

    def run(self):
        """
        运行主方法
        :return:
        """
        self.compute_or_improve_path()
        self.publish_path()

        while True:
            if self.eps <= 1:
                break

            self.eps -= 0.5
            self.OPEN.update(self.INCONS)
            for s in self.OPEN:
                self.OPEN[s] = self.get_key(s)
            self.CLOSED = set()
            self.compute_or_improve_path()
            self.publish_path()


def main():
    """
    main func
    :return:
    """


if __name__ == '__main__':
    main()
