import math

import numpy as np

from algorithm.potential_field.PotentialField import PotentialField
from data import obstacles
from data.Map import Map


class PotentialFieldWithRegression(PotentialField):
    def __init__(self, env, start, goal, k_att, k_rep, rr, max_iter, coefficient, dis_obs_goal, dis_goal_current):
        super().__init__(env, start, goal, k_att, k_rep, rr, max_iter)

        self.coefficient = coefficient
        self.dis_goal_current = dis_goal_current
        self.dis_obs_goal = dis_obs_goal
        self.FLAG = 'Potential Field with Regression '

    def get_attractive_force_rs(self, start, goal):
        """
        改进之后引力大小的计算
        :return: 引力对应矢量
        """
        dis_goal = math.hypot(start[0] - goal[0], start[1] - goal[1])
        force = self.k_att * np.subtract(goal, start)
        if dis_goal <= self.coefficient:
            return force
        else:
            return self.coefficient / dis_goal * force

    def get_repulsion_force_rs(self, goal):
        force = np.zeros(2)
        for obs in self.env.obs:
            dis_obs = math.hypot(self.current_pos[0] - obs[0], self.current_pos[1] - obs[1])
            dis_goal = math.hypot(self.current_pos[0] - goal[0], self.current_pos[1] - goal[1])
            if dis_obs <= self.rr:
                rep_1 = np.subtract(self.current_pos, np.asarray(obs)) * self.k_rep \
                        * (1.0 / dis_obs - 1.0 / self.rr) / (dis_obs ** 2) * (dis_goal ** 2)
                rep_2 = np.subtract(goal, np.asarray(obs)) * self.k_rep * (
                            (1.0 / dis_obs - 1.0 / self.rr) ** 2) * dis_goal
                force = np.add(rep_1, rep_2, force)

        return force

    def get_total_force_rs(self, goal):
        """
        改进计算合力方式
        :return:
        """
        min_dis = float('inf')
        min_obs = 0
        for obs in self.env.obs:
            if math.hypot(self.current_pos[0] - obs[0], self.current_pos[1] - obs[1]) < min_dis:
                min_obs = obs
        dis_qc = math.hypot(min_obs[0] - goal[0], min_obs[1] - goal[1])
        dis_gr = math.hypot(self.current_pos[0] - goal[0], self.current_pos[1] - goal[1])
        if dis_qc <= self.dis_obs_goal and dis_gr <= self.dis_goal_current:
            return self.get_attractive_force_rs(self.current_pos, goal)
        else:
            return np.add(self.get_attractive_force_rs(self.current_pos, goal), self.get_repulsion_force_rs(goal))

    def check_obs_pos(self, goal):
        """
        查找垂于与当前点与goal连线的线上的障碍物
        :param goal:
        :return:
        """
        # obs_list[0] y值大于等于当前点y值， obs_list[1] y值小于等于当前点y值
        obs_list = [(float('inf'), float('inf')), (float('-inf'), float('-inf'))]
        length = 2
        if self.current_pos[1] - goal[1] == 0:
            x = self.current_pos[0]
            for y in range(self.env.y_range):
                node = (round(x), y)
                if node in self.env.obs:
                    if self.current_pos[1] < y < obs_list[0][1]:
                        obs_list[0] = node
                    if obs_list[1][1] < y < self.current_pos[1]:
                        obs_list[1] = node
        else:
            k = - (self.current_pos[0] - goal[0]) / (self.current_pos[1] - goal[1])
            b = self.current_pos[1] - k * self.current_pos[0]
            bias = abs(k / 2)
            # 找到路径上障碍物
            for x in range(self.env.x_range):
                nodes = set()
                nodes.add((x, round(k * x + b - bias)))
                nodes.add((x, round(k * x + b + bias)))
                nodes.add((x, math.ceil(k * x + b)))
                nodes.add((x, math.floor(k * x + b)))
                for node in nodes:
                    if node in self.env.obs:
                        if k >= 0:
                            if self.current_pos[0] < node[0] < obs_list[0][0]:
                                obs_list[0] = node
                            if obs_list[1][0] < node[0] < self.current_pos[0]:
                                obs_list[1] = node
                        else:
                            if self.current_pos[1] < node[1] < obs_list[0][1]:
                                obs_list[0] = node
                            if obs_list[1][1] < node[1] < self.current_pos[1]:
                                obs_list[1] = node
        if obs_list[0] == (float('inf'), float('inf')):
            length -= 1
        if obs_list[1] == (float('-inf'), float('-inf')):
            length -= 1
        return length

    def dfs_obs(self, obs_pos):
        stack = [obs_pos]
        seen = [obs_pos]
        directions = {obs_pos: [obs_pos, obs_pos]}
        results = []

        while stack:
            v_node = stack.pop()
            is_colon = True
            for motion in Map.MOTIONS["manhattan"]:
                s_next = tuple([v_node[i] + motion[i] for i in range(2)])
                if s_next in self.env.obs and s_next not in seen:
                    stack.append(s_next)
                    seen.append(s_next)
                    directions[s_next] = [motion, v_node]
                    is_colon = False
            if is_colon:  # 如果找到了一个尽头
                cur_node = v_node
                cur_motion = directions[cur_node][0]
                pre_node = directions[cur_node][1]
                pre_motion = directions[pre_node][0]
                line = []
                node = (cur_node[0] + cur_motion[0], cur_node[1] + cur_motion[1])
                if node not in self.env.obs and (0 < node[0] < self.env.x_range) and (0 < node[1] < self.env.y_range):
                    line.append(node)
                while pre_node is not obs_pos:
                    if cur_motion != pre_motion:
                        node = (pre_node[0] + pre_motion[0], pre_node[1] + pre_motion[1])
                        if node not in self.env.obs and (0 < node[0] < self.env.x_range) \
                                and (0 < node[1] < self.env.y_range):
                            line.append(node)
                    cur_node = pre_node
                    cur_motion = directions[cur_node][0]
                    pre_node = directions[cur_node][1]
                    pre_motion = directions[pre_node][0]
                if line:
                    results.append(line)

        return results

    def get_point_dis_line(self, point, goal):
        vec1 = np.subtract(goal, np.asarray(point))
        vec2 = np.subtract(self.current_pos, np.asarray(point))
        distance = np.abs(np.cross(vec1, vec2) / np.linalg.norm(np.subtract(goal, self.current_pos)))
        return distance

    def deal_minima_problem(self, obs_pos, goal):
        """
        处理局部最小值问题
        :obs_pos: 陷入障碍物的坐标
        :return: 返回虚拟目标点序列
        """
        result = []
        size = self.check_obs_pos(goal)
        paths = self.dfs_obs(obs_pos)
        if size == 0:
            dis = []
            for path in paths:
                dis.append(self.get_point_dis_line(path[0], goal))
            idx = dis.index(min(dis))
            result.append(paths[idx][0])
        elif size == 1:
            dis = []
            for path in paths:
                dis_path = 0
                for idx, node in enumerate(path):
                    if idx == 0:
                        dis_path += math.hypot(self.current_pos[0] - node[0], self.current_pos[1] - node[1])
                    if idx == len(path) - 1:
                        dis_path += math.hypot(goal[0] - node[0], goal[1] - node[1])
                    if idx < len(path) - 1:
                        dis_path += math.hypot(node[0] - path[idx + 1][0], node[1] - path[idx + 1][1])
                dis.append(dis_path)
            idx = dis.index(min(dis))
            result.extend(paths[idx])
        elif size == 2:
            dis = []
            for path in paths:
                dis.append(self.get_point_dis_line(path[0], goal))
            idx = dis.index(min(dis))
            result.extend(paths[idx])
        return result

    def has_obs_in_path(self, start, goal):
        result = False
        if goal[0] - start[0] == 0:
            if start[1] > goal[1]:
                min_y = goal[1]
                max_y = start[1]
            else:
                min_y = start[1]
                max_y = goal[1]
            for y in range(int(min_y), int(max_y)):
                node = (goal[0], y)
                if node in self.env.obs:
                    result = True
        else:
            k = (goal[1] - start[1]) / (goal[0] - start[0])
            b = goal[1] - k * goal[0]
            bias = abs(k/2)
            if start[0] > goal[0]:
                min_x = goal[0]
                max_x = start[0]
            else:
                min_x = start[0]
                max_x = goal[0]
            for x in range(int(min_x), int(max_x)):
                nodes = set()
                nodes.add((x, round(k * x + b - bias)))
                nodes.add((x, round(k * x + b + bias)))
                nodes.add((x, math.ceil(k * x + b)))
                nodes.add((x, math.floor(k * x + b)))
                for node in nodes:
                    if node in self.env.obs:
                        result = True
        return result

    def get_path_apf(self, start, target):
        pos = start
        path = [(pos[0], pos[1])]
        while not np.array_equal(pos, target):
            force = self.get_attractive_force_rs(pos, target)
            pos = self.get_next_pos(pos, force)
            path.append((pos[0], pos[1]))
        return path

    def optimize_path_by_regression(self):
        """
        基于回归的方法进行路径优化
        :return:
        """
        cur_end = 1
        cur_start = cur_end - 1
        path = [self.path[0]]
        while cur_start < len(self.path):
            start = self.path[cur_start]
            end = self.path[cur_end]
            if self.has_obs_in_path(start, end):
                target = self.path[cur_end - 1]
                sub_path = self.get_path_apf(np.asarray(start), np.asarray(target))
                path.extend(sub_path[1:])
                cur_start = cur_end - 1
            else:
                if cur_end == len(self.path) - 1:
                    if start not in path:
                        path.append(start)
                    cur_start += 1
                else:
                    cur_end += 1
        return path

    def plan_path_rs(self, goal):
        self.is_success = True
        count = 0
        while not np.array_equal(self.current_pos, goal):
            if count > self.max_iter:
                self.is_success = False
                break
            count += 1
            force = self.get_total_force_rs(goal)
            pos = self.get_next_pos(self.current_pos, force)
            # 触发局部最小值处理
            if (pos[0], pos[1]) in self.env.obs:
                targets = self.deal_minima_problem((pos[0], pos[1]), goal)
                for target in targets:
                    self.plan_path_rs(np.asarray(target))
            else:
                self.current_pos = pos
                self.path.append((self.current_pos[0], self.current_pos[1]))

    def plan_path(self):
        """
        改进之后的运行流程
        :return:
        """
        self.plan_path_rs(self.goal)
        self.path = self.optimize_path_by_regression()


def main():
    # basic
    start = (5, 5)
    goal = (45, 25)
    k_att = 1.0
    k_rep = 0.8
    rr = 3
    max_iter = 1000
    # 主函数
    heuristic_type = "euclidean"
    env = Map(51, 31, heuristic_type=heuristic_type)
    obs, free = obstacles.get_anytime_standard_obs(env.x_range, env.y_range)
    env.update_obs(obs, free)
    # improved
    coefficient = 30
    dis_goal_obs = 4
    dis_goal_current = 6
    demo = PotentialFieldWithRegression(env, start, goal, k_att, k_rep, rr, max_iter,
                                        coefficient, dis_goal_obs, dis_goal_current)
    demo.run()


if __name__ == "__main__":
    main()
