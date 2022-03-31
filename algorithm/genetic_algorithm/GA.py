import random

from algorithm.genetic_algorithm.Population import create_graph


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
        paths = set()
        graph = create_graph(self.env.free, self.start, self.goal)
        for i in range(self.population_max):
            path = [self.goal]
            node = self.goal
            while node != self.start:
                edges = graph[node]
                node = edges[random.randint(0, len(edges))]
                path.insert(node, 0)
            paths.add(path)
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
        pass


def main():
    pass


if __name__ == "__main__":
    main()
