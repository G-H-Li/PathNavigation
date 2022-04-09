import math
import random
import time

from bresenham import bresenham

from data import obstacles
from data.Map import Map
from data.plotting import plot_clear, plot_set_title, plot_map, plot_path, plot_show


class GA:
    def __init__(self, env, start, goal, max_iter, population_max, crossover_rate, mutation_rate):
        self.env = env
        self.start = start
        self.goal = goal
        self.max_iter = max_iter
        self.population_max = population_max
        self.crossover_rate = crossover_rate
        self.mutation_rate = mutation_rate
        self.population = []

        self.pre_best_path = []
        self.generation = 1
        self.stop_criteria = 0

        self.is_success = False
        self.FLAG = "Genetic Algorithm:"

    def create_graph(self, start, goal):
        graph = {start: []}
        node = start
        while not self.is_feasible(node, goal):
            s_node, e_node = self.new_feasible_edge(graph)
            if s_node is None:
                continue
            graph[e_node] = [s_node]
            u_set = list(graph.keys())
            u_set.remove(s_node)
            for idx in range(len(u_set)):
                u_set[idx] = list(u_set[idx])
                u_set[idx].append(math.hypot(u_set[idx][0] - e_node[0], u_set[idx][1] - e_node[1]))
            u_set.sort(key=lambda el: el[2])
            for ele in u_set:
                if self.is_feasible((ele[0], ele[1]), e_node):
                    graph[e_node].append((ele[0], ele[1]))
            node = e_node
        graph[goal] = [node]

        return graph

    def new_feasible_edge(self, graph):
        nodes = list(graph.keys())
        start = nodes[random.randint(0, len(nodes) - 1)]
        free_nodes = list(self.env.free.difference(set(nodes)))
        end = free_nodes[random.randint(0, len(free_nodes) - 1)]
        path_nodes = list(bresenham(start[0], start[1], end[0], end[1]))

        idx = len(path_nodes) - 1
        while idx > 0:
            end = path_nodes[idx]
            if self.is_feasible(start, end):
                return start, end
            else:
                idx -= 1

        return None, None

    def is_feasible(self, start, goal):
        path = list(bresenham(start[0], start[1], goal[0], goal[1]))
        if start == goal:
            return False
        for node in path:
            if node in self.env.obs:
                return False
        return True

    def generate_population(self):
        paths = []
        graph = self.create_graph(self.start, self.goal)
        for i in range(self.population_max):
            path = [self.goal]
            node = self.goal
            while node != self.start:
                edges = graph[node]
                node = edges[random.randint(0, len(edges)-1)]
                path.insert(0, node)
            paths.append(path)
        return paths

    def get_path_len(self, path):
        """
        计算path的长度
        :return: 路径长度
        """
        length = 0
        for idx, node in enumerate(path):
            if idx > 0:
                if self.is_feasible(path[idx-1], node):
                    length += math.hypot(node[0] - path[idx - 1][0], node[1] - path[idx - 1][1])
                else:
                    length += self.env.x_range

        return length

    def get_fitness(self, paths):
        fitness = {}
        for idx, path in enumerate(paths):
            fitness[idx] = self.get_path_len(path)

        return sorted(fitness.items(), key=lambda kv: (kv[1], kv[0]))

    def selector(self, fitness):
        select_num = int(len(fitness) * (1 - self.crossover_rate))
        truncate_once = select_num//3-1
        truncate_2 = len(fitness) // 3
        truncate_3 = len(fitness)*2//3

        elite = fitness[0: select_num]
        truncate = []
        truncate.extend(fitness[0: truncate_once])
        truncate.extend(fitness[truncate_2: truncate_2+truncate_once])
        truncate.extend(fitness[truncate_3: truncate_3+truncate_once])

        return elite, truncate

    def cross_over(self, elite, tunc):
        results = []
        m = 0
        while m < int(self.population_max * self.crossover_rate):
            children = []
            path1 = elite[random.randint(0, len(elite)-1)]
            path2 = tunc[random.randint(0, len(tunc)-1)]
            cross_point_list = []
            match_point_list = []

            for i in range(1, len(path1)-1):
                for j in range(1, len(path2)-1):
                    if self.is_feasible(path1[i+1], path2[j]) and self.is_feasible(path2[j+1], path1[i]):
                        cross_point_list.append(i)
                        match_point_list.append(j)

            for k in range(len(cross_point_list)):
                off_spring1 = list.copy(path1)
                off_spring2 = list.copy(path2)
                off_spring1[cross_point_list[k]] = path2[match_point_list[k]]
                off_spring2[match_point_list[k]] = path1[cross_point_list[k]]
                children.append(off_spring1)
                children.append(off_spring2)

            fitness = self.get_fitness(children)
            if len(fitness) >= 2:
                results.extend([children[fitness[0][0]], children[fitness[1][0]]])
                m += 2
            elif len(fitness) == 1:
                results.append(children[fitness[0][0]])
                m += 1

        return results

    def get_neighbor(self, s):
        """
        返回当前点可访问的临近点
        :param s: node
        :return: 集合
        """
        neighbors = set()
        for u in self.env.motions:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.env.obs and \
                    0 < s_next[0] < self.env.x_range - 1 and \
                    0 < s_next[1] < self.env.y_range - 1:
                neighbors.add(s_next)
        return neighbors

    def mutation(self, gene):
        results = list.copy(gene)
        for idx, child in enumerate(results):
            if random.random() < self.mutation_rate:
                node_idx = random.randint(1, len(child) - 2)
                node = child[node_idx]
                neighbors = self.get_neighbor(node)
                mini_fitness = self.get_path_len(child)
                for neighbor in neighbors:
                    if self.is_feasible(child[node_idx-1], neighbor) and self.is_feasible(neighbor, child[node_idx+1]):
                        child[node_idx] = neighbor
                        if self.get_path_len(child) < mini_fitness:
                            results[idx][node_idx] = neighbor
        return results

    def do_first_ga(self):
        fitness = self.get_fitness(self.population)
        elite_gene, tunc_gene = self.selector(fitness)
        elite_population = [self.population[gene[0]] for gene in elite_gene]
        tunc_population = [self.population[gene[0]] for gene in tunc_gene]
        next_generation = list.copy(elite_population)
        next_generation.extend(self.cross_over(elite_population, tunc_population))
        return self.mutation(next_generation), fitness[0]

    def publish_path(self, path):
        result = [path[0]]
        for idx in range(len(path)-1):
            node = path[idx]
            next_node = path[idx+1]
            if next_node not in self.get_neighbor(node):
                sub_path = list(bresenham(node[0], node[1], next_node[0], next_node[1]))
                result.extend(sub_path[1:])
            else:
                result.append(next_node)
        return result

    def explore_path(self):
        plot_clear()
        plot_set_title(self.FLAG)
        plot_map(self.env.obs)

        start_time = time.time()
        path_cost = 0
        self.population = self.generate_population()
        next_population, best_fitness = self.do_first_ga()
        while not self.is_success:
            self.pre_best_path = self.population[best_fitness[0]]
            path_cost = best_fitness[1]
            self.population = next_population
            self.generation += 1
            next_population, best_fitness = self.do_first_ga()

            if self.pre_best_path == self.population[best_fitness[0]]:
                self.stop_criteria += 1
            else:
                self.stop_criteria = 0

            if self.stop_criteria >= 5:
                self.is_success = True

        end_time = time.time()
        self.pre_best_path = self.publish_path(self.pre_best_path)
        plot_path(self.pre_best_path, self.start, self.goal)
        print(self.FLAG, "generation:", self.generation, "path length:", path_cost,
              "cost time:", end_time-start_time, 's')

    def run(self):
        self.explore_path()
        plot_show()


def main():
    # basic
    start = (1, 1)
    goal = (20, 20)
    max_iter = 1000
    population_max = 100
    mutation_rate = 0.3
    crossover_rate = 0.5
    # 主函数
    heuristic_type = "euclidean"
    env = Map(21, 21, heuristic_type=heuristic_type)
    obs, free = obstacles.get_rough_obs(env.x_range, env.y_range)
    env.update_obs(obs, free)

    demo = GA(env, start, goal, max_iter, population_max, crossover_rate, mutation_rate)
    demo.run()


if __name__ == "__main__":
    main()
