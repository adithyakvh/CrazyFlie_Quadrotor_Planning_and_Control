from heapq import heappush, heappop  # Recommended.
import numpy as np

from flightsim.world import World

from collections import defaultdict
from .occupancy_map import OccupancyMap  # Recommended.

from .occupancy_map import OccupancyMap  # Recommended.


def find_adjacent_cubes(index_value, graph_shape, occ_map):
    graph_shape = np.array([graph_shape])

    all_combinations = np.array(
        [[-1, -1, -1], [-1, -1, 0], [-1, -1, 1], [-1, 0, -1], [-1, 0, 0], [-1, 0, 1], [-1, 1, -1],
         [-1, 1, 0], [-1, 1, 1], [0, -1, -1], [0, -1, 0], [0, -1, 1], [0, 0, -1], [0, 0, 1], [0, 1, -1], [0, 1, 0],
         [0, 1, 1], [1, -1, -1], [1, -1, 0], [1, -1, 1], [1, 0, -1], [1, 0, 0], [1, 0, 1], [1, 1, -1], [1, 1, 0],
         [1, 1, 1]])

    all_neighbours = index_value + all_combinations[:, :]

    all_neighbours = all_neighbours[np.all(all_neighbours >= 0, axis=1), :]

    all_neighbours = all_neighbours[np.all(all_neighbours < graph_shape, axis=1), :]
    x_columns = all_neighbours[:, 0]
    y_columns = all_neighbours[:, 1]
    z_columns = all_neighbours[:, 2]

    all_neighbours = all_neighbours[np.where(occ_map[x_columns, y_columns, z_columns] == False)]

    return all_neighbours


def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        return a tuple (path, nodes_expanded)
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
        nodes_expanded, the number of nodes that have been expanded
    """

    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)

    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))

    distance_f_dict = defaultdict(lambda: float("inf"))
    distance_g_dict = defaultdict(lambda: float("inf"))
    parent_dict = {}

    graph_shape = occ_map.map.shape

    Q_list = [(0, start_index)]
    Y_set = set()

    distance_f_dict[start_index] = 0
    distance_g_dict[start_index] = np.linalg.norm((np.array(goal_index) - np.array(start_index)))

    no_of_nodes = 0

    while Q_list:

        closest_node = heappop(Q_list)[1]
        if closest_node == goal_index:
            path = []
            val_out = goal_index
            path.append(goal)
            while val_out is not None:
                # try:
                parent = parent_dict[val_out]
                if parent == start_index:
                    path.insert(0, start)
                    break
                path.insert(0, occ_map.index_to_metric_center(parent))
                val_out = parent

            path = np.array(path)
            return (path, no_of_nodes)

        no_of_nodes += 1
        Y_set.add(closest_node)
        neighbour_nodes = find_adjacent_cubes(closest_node, graph_shape, occ_map.map)
        # print("neighbour_nodes", neighbour_nodes)


        for each_neighbour in neighbour_nodes:
            each_neighbour_tupe = tuple(each_neighbour)
            if each_neighbour_tupe in Y_set:
                continue

            distance = np.linalg.norm((np.array(each_neighbour) - np.array(closest_node)))
            # distance = np.linalg.norm(neighbour_nodes - np.array(closest_node), axis = 1).reshape(-1, 1)
            # distance = np.sqrt(np.sum(np.square(neighbour_nodes - np.array(closest_node)), axis=1).reshape(-1, 1))
            # print("DIST HERE", distance)
            distance_g = distance_g_dict[closest_node] + (distance)
            distance_f = distance_g
            if astar:
                distance_f = distance_g + np.linalg.norm((np.array(goal_index) - np.array(each_neighbour)))
                # distance = distance + np.sqrt(np.sum(np.square(goal_index-each_neighbour)))

            if distance_g < distance_g_dict[each_neighbour_tupe]:
                distance_g_dict[each_neighbour_tupe] = distance_g
                distance_f_dict[each_neighbour_tupe] = distance_f
                parent_dict[each_neighbour_tupe] = closest_node
                heappush(Q_list, (distance_f, each_neighbour_tupe))

    return None, no_of_nodes