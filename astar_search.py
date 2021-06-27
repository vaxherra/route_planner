from queue import PriorityQueue
from math import sqrt


def shortest_path(M, start, goal):
    """
    Finds the shortest path from start to goal node for a given map M using "A*" algorithm, using euclidean distance
    as a measure for the H function, i.e. estimated distance to the goal.

    :param M: a map
    :type M: Map object

    :param start: start node number
    :type start: int

    :param goal: end/goal node number
    :type goal: int

    :return: list with the shortest path of nodes from start->goal (or None if not available)
    :rtype: list/None
    """

    # The lowest valued entries are retrieved first (based on a first value in the tuple)
    # https://docs.python.org/3/library/queue.html
    frontier = PriorityQueue()

    # To trace back a path from goal, to start
    # dict holds: for a given node, a previous node with min path cost
    prev_link = {start: None}

    # dict holding path cost (g),
    # growing sets of keys are indicator of the "explored" nodes
    G = {start: 0}

    # Calculate Euclidean distances for each node between the goal node
    # has required property - never overestimates the distance to the goal,
    # this is because in a 2D plane, the Euclidean distance is going to be at most EQUAL to the distance of the 2d map
    H = dict()  # dict holding all distances
    for node in M.intersections.keys():
        H[node] = euc_dist(M, node, goal)

    # Initialize a pririty queue with start node and its f value = g + h = 0 + h
    frontier.put((H[start]+0, start))

    # main loop
    while not frontier.empty():

        # get a node from queue with min f value
        _, current_node = frontier.get()

        if current_node == goal:
            reconstruct_path(prev_link, start, goal)

        # Get all frontiers of current-node
        for neighbor in M.roads[current_node]:

            # g_tentative is the path cost of neighbor from start through current_node
            g_tentative = G[current_node] + euc_dist(M, current_node, neighbor)

            # Proceed only if neighbor is not explored or its explored then
            # g_tentative value should be less than G[neighbor]
            if neighbor not in G or g_tentative < G[neighbor]:
                G[neighbor] = g_tentative

                # f = g + h
                f = g_tentative + H[neighbor]

                # add it to frontier_node
                frontier.put((f, neighbor))

                # link all neighbors to the current_node (i.e. prev_node)
                prev_link[neighbor] = current_node

    return reconstruct_path(prev_link, start, goal)


# Calculate distance (d)
def euc_dist(M, node1, node2):
    """Calculate Euclidean distance between two nodes for a given map

    :param M: a map
    :type M: Map object

    :param node1: node name
    :type node1: int

    :param node2:
    :type node2: int

    :return: an Euclidean distance between node1 and node2 given a map M
    :rtype: float
    """
    # each node is described by a tuple, containing x,y coordinates
    x1, y1 = M.intersections[node1]
    x2, y2 = M.intersections[node2]

    # a simple euclidean distance
    dist = sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    return dist


# reconstruct path from prev link ones
def reconstruct_path(prev_link, start, goal):
    """A helper function to reconstruct a shortest path from the start to the goal nodes, given a dictionary holding
    previous nodes with the smallest cost toward the goal (values of dict) for a given node (key)

    :param prev_link: a dictionary backtracking nodes with smallest costs
    :param start: starting node
    :param goal: goal node

    :return: list of nodes from the start to goal node with the smallest cost
    :rtyp: list
    """

    # backtracking from goal -> start, and at the end, reversing the path
    path = [goal]

    curr = goal  # curr, holds currently tracked nodes
    while curr != start:
        curr = prev_link.get(curr, None)  # None for when node is not present
        if curr is None:
            return None
        path.append(curr)

    # reverse the backtracked solution
    return path[::-1]
