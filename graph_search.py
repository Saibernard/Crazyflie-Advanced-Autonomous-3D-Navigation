from heapq import heappush, heappop  # Recommended.
import numpy as np

import heapq
import itertools

from flightsim.world import World

from .occupancy_map import OccupancyMap # Recommended.

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
    # print('start position', start)
    # print('start index', start_index)
    dist = np.ones_like(occ_map.map) * float('inf')
    parents = np.empty_like(occ_map.map, dtype=object) # this is needed for storing a tuple
    dist[start_index] = 0
    hq = [[0, start_index]]
    heapq.heapify(hq)
    num_nodes = 0
    is_possible = False
    while hq:
        curr = heappop(hq)
        if curr[0] == float('inf'):
            # print("no path available, exiting...")
            break
        if curr[1] == goal_index:
            # print('distance is', curr[0])
            # print("path found!")
            is_possible = True
            break
        # actions = [[-1,0,0], [1,0,0],\
        #            [0,-1,0], [0,1,0],\
        #            [0,0,-1], [0,0,1]]

        # # corners
        # actions = [[-1,0,0], [1,0,0],\
        #            [0,-1,0], [0,1,0],\
        #            [0,0,-1], [0,0,1],\
        #            [-1,-1,-1], [-1,-1,1],\
        #            [-1,1,-1],[-1,1,1],\
        #            [1,-1,-1],[1,-1,1],\
        #            [1,1,-1],[1,1,1]]

        actions = [[-1,0,0], [1,0,0],\
                   [0,-1,0], [0,1,0],\
                   [0,0,-1], [0,0,1],\
                   # [-1,-1,-1], [-1,-1,1],\
                   # [-1,1,-1],[-1,1,1],\
                   # [1,-1,-1],[1,-1,1],\
                   # [1,1,-1],[1,1,1],\
                   [0,-1,-1],[0,-1,1],[0,1,-1],[0,1,1],\
                   [-1,0,-1],[-1,0,1],[1,0,-1],[1,0,1],\
                   [-1,-1,0],[-1,1,0],[1,-1,0],[1,1,0]]

        for i in range(len(actions)):
            action = actions[i]
            neighbor_idx = (curr[1][0]+action[0], curr[1][1]+action[1], curr[1][2]+action[2])
            if occ_map.is_occupied_index(neighbor_idx):
                continue

            if i < 6:
                new_dist = dist[curr[1]] + 1 #np.linalg.norm(action)
            else:
                new_dist = dist[curr[1]] + np.sqrt(2)

            num_nodes += 1
            if new_dist < dist[neighbor_idx]:
                dist[neighbor_idx] = new_dist
                parents[neighbor_idx] = curr[1]

                if astar:
                    heuristic = np.linalg.norm([neighbor_idx[0]-goal_index[0],neighbor_idx[1]-goal_index[1],neighbor_idx[2]-goal_index[2]])
                    heappush(hq, [new_dist + heuristic, neighbor_idx])
                else:
                    heappush(hq, [new_dist, neighbor_idx])

    if not is_possible:
        return None, num_nodes

    path = []
    curr = goal_index
    while curr:
        pt = occ_map.index_to_metric_center(curr)
        path.append(pt)
        curr = parents[curr]

    path.reverse()
    path[0] = start
    path[-1] = goal

    return np.array(path), num_nodes
