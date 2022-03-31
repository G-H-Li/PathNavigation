import math
import random

from bresenham import bresenham


def create_graph(free, start, goal):
    graph = {start: []}
    node = start
    while not _is_feasible(free, node, goal):
        s_node, e_node = _new_feasible_edge(free, graph)
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
            if _is_feasible(free, (ele[0], ele[1]), e_node):
                graph[e_node].append((ele[0], ele[1]))
        node = e_node
    graph[goal] = [node]

    return graph


def _new_feasible_edge(free, graph):
    nodes = list(graph.keys())
    start = nodes[random.randint(0, len(nodes)-1)]
    free_nodes = list(free.difference(set(nodes)))
    end = free_nodes[random.randint(0, len(free_nodes)-1)]
    path_nodes = list(bresenham(start[0], start[1], end[0], end[1]))

    idx = len(path_nodes) - 1
    while idx > 0:
        end = path_nodes[idx]
        if _is_feasible(free, start, end):
            return start, end
        else:
            idx -= 1

    return None, None


def _is_feasible(free, start, goal):
    result = True
    path = list(bresenham(start[0], start[1], goal[0], goal[1]))
    for node in path:
        if node not in free:
            result = False
    return result

