import math


def create_graph(free, start, goal):
    graph = {start: []}
    node = start
    while not _is_feasible(free, node, goal):
        

    return graph


def _new_feasible_edge():
    pass


def _is_feasible(env, start, goal):
    result = True
    if goal[0] - start[0] == 0:
        if start[1] > goal[1]:
            min_y = goal[1]
            max_y = start[1]
        else:
            min_y = start[1]
            max_y = goal[1]
        for y in range(int(min_y), int(max_y)):
            node = (goal[0], y)
            if node not in env:
                result = False
    else:
        k = (goal[1] - start[1]) / (goal[0] - start[0])
        b = goal[1] - k * goal[0]
        bias = abs(k / 2)
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
                if node not in env:
                    result = False
    return result


def _bresenham(start, end):
    pass
