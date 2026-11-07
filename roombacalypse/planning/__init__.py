"""Getting from here to there on the robot's own map, and covering all of it (belief side).

* :mod:`costmap`: the SLAM occupancy grid turned into where the robot's centre may go, and at what cost;
* :mod:`astar`: shortest paths (A*, and Dijkstra fields for one-to-many questions), with path pruning;
* :mod:`coverage`: what to clean next: boustrophedon lanes, backtracking (BA*), frontier exploration.
"""
