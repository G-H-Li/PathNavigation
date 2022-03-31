import random

from algorithm.genetic_algorithm.Population import create_graph
from data import obstacles
from data.Map import Map


class GA:
    def __init__(self, env, start, goal, max_iter, population_max, mutation_rate):
        self.env = env
        self.start = start
        self.goal = goal
        self.max_iter = max_iter
        self.population_max = population_max
        self.mutation_rate = mutation_rate

        self.is_success = True
        self.FLAG = "Genetic Algorithm:"

    def population(self):
        paths = []
        graph = create_graph(self.env.free, self.start, self.goal)
        for i in range(self.population_max):
            path = [self.goal]
            node = self.goal
            while node != self.start:
                edges = graph[node]
                node = edges[random.randint(0, len(edges)-1)]
                path.insert(0, node)
            paths.append(path)
            print(path)
        return paths

    def get_fitness(self):
        pass

    def selector(self):
        pass

    def cross_over(self):
        pass

    def mutation(self):
        pass

    def explore_path(self):
        pass

    def run(self):
        self.population()


def main():
    # basic
    start = (5, 5)
    goal = (45, 25)
    max_iter = 1000
    population_max = 1000
    mutation_rate = 0.01
    # 主函数
    heuristic_type = "euclidean"
    env = Map(51, 31, heuristic_type=heuristic_type)
    obs, free = obstacles.get_anytime_standard_obs(env.x_range, env.y_range)
    env.update_obs(obs, free)

    demo = GA(env, start, goal, max_iter, population_max, mutation_rate)
    demo.run()


if __name__ == "__main__":
    main()
