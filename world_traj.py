import numpy as np

from .graph_search import graph_search
# import occupancy_map
# import se3_control

class WorldTraj(object):
    """

    """
    def __init__(self, world, start, goal):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. You
        are free to choose the path taken in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Parameters:
            world, World object representing the environment obstacles
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)

        """

        # You must choose resolution and margin parameters to use for path
        # planning. In the previous project these were provided to you; now you
        # must chose them for yourself. Your may try these default values, but
        # you should experiment with them!
        self.resolution = np.array([0.05, 0.05, 0.05])
        self.margin = 0.20

        # You must store the dense path returned from your Dijkstra or AStar
        # graph search algorithm as an object member. You will need it for
        # debugging, it will be used when plotting results.
        self.path, _ = graph_search(world, self.resolution, self.margin, start, goal, astar=True)

        # You must generate a sparse set of waypoints to fly between. Your
        # original Dijkstra or AStar path probably has too many points that are
        # too close together. Store these waypoints as a class member; you will
        # need it for debugging and it will be used when plotting results.
        self.points = np.zeros((1,3)) # shape=(n_pts,3)

        # Finally, you must compute a trajectory through the waypoints similar
        # to your task in the first project. One possibility is to use the
        # WaypointTraj object you already wrote in the first project. However,
        # you probably need to improve it using techniques we have learned this
        # semester.

        # STUDENT CODE HERE
        self.waypoints = self.path
        self.directions = np.zeros((self.waypoints.shape[0] - 1, self.waypoints.shape[1]))
        self.segment_distances = np.zeros((self.directions.shape[0], 1))
        self.desired_velocities = np.zeros((self.directions.shape[0], self.waypoints.shape[1]))
        self.start_times = np.zeros((self.directions.shape[0], 1))

        self.velocity = 1.0
        time_elapsed = 0
        for i in range(len(self.waypoints) - 1):
            point_diff = self.waypoints[i + 1] - self.waypoints[i]
            distance = np.linalg.norm(point_diff)
            self.segment_distances[i] = distance
            self.directions[i] = point_diff / distance
            self.desired_velocities[i] = self.velocity * self.directions[i]
            time_elapsed += distance / self.velocity
            self.start_times[i] = time_elapsed

        self.desired_velocities = np.vstack([self.desired_velocities, np.zeros((1, 3))])
        self.directions = np.vstack([self.directions, np.zeros((1, 3))])
        self.segment_distances = np.vstack([self.segment_distances, np.zeros((1,))])
        self.directions = np.vstack([self.directions, np.zeros((1, 3))])
        self.start_times = np.insert(self.start_times, 0, 0)

    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        x        = np.zeros((3,))
        x_dot    = np.zeros((3,))
        x_ddot   = np.zeros((3,))
        x_dddot  = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # STUDENT CODE HERE
        if t >= self.start_times[-1]:
            x_dot = np.zeros((3,))
            x = self.waypoints[-1]
        else:
            for i in range(len(self.start_times)):
                if self.start_times[i] <= t < self.start_times[i + 1]:
                    x_dot = self.velocity * self.directions[i]
                    x = self.waypoints[i] + x_dot * (t - self.start_times[i])
                    break

        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output
