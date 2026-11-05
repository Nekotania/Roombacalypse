"""What the world looks like, as far as the robot knows (belief side).

* :mod:`occupancy_grid`: the log-odds occupancy grid the robot builds from its scans;
* :mod:`likelihood_field`: smooth "how close to a mapped surface" fields that scans are matched against;
* :mod:`pose_graph`: the SLAM back end, a sparse nonlinear least-squares pose-graph optimiser.

Like everything on the belief side, this package learns about the world only
through :class:`~roombacalypse.core.messages.SensorFrame` contents.
"""
