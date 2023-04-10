import numpy as np
from scipy.sparse.linalg import spsolve
from scipy.sparse import SparseEfficiencyWarning
from scipy.sparse import SparseWarning
from scipy.sparse import lil_matrix
import scipy
from .graph_search import graph_search


def _vec(pt1, pt2):

    diff = pt2-pt1
    unit_vec = diff/np.linalg.norm(diff)
    return unit_vec


def straighten_path(points):

    path_points = list(points)
    track = 0

    while track != len(path_points)-2:
        if track > len(path_points)-2:
            break
        pt1 = path_points[track]
        pt2 = path_points[track + 1]
        pt3 = path_points[track + 2]
        vec1 = _vec(pt2, pt1)
        vec2 = _vec(pt3, pt2)
        try:
            # angle = np.degrees(np.arccos(np.dot(vec1, vec2)))
            angle = np.linalg.norm(np.cross(vec1, vec2))
        except:
            angle = 0
        distance = np.linalg.norm(pt1 - pt2)
        if angle == 0:
            del path_points[track+1]
            track -= 1
        elif distance > 0.01:
            del path_points[track]
        track += 1

    path_points = np.delete(path_points, 1, 0)
    return path_points


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
        self.resolution = np.array([0.255, 0.25, 0.25])
        self.world = world
        self.margin = 0.5
        # You must store the dense path returned from your Dijkstra or AStar
        # graph search algorithm as an object member. You will need it for
        # debugging, it will be used when plotting results.
        self.path, _ = graph_search(world, self.resolution, self.margin, start, goal, astar=True)

        self.points = straighten_path(self.path)
        # self.points = collision_pruning(self.points, world)
        self.v = 6.15
        # self.points = self.path

        self.directions = np.zeros((self.points.shape[0] - 1, self.points.shape[1]))
        self.distance_segment = np.zeros((self.directions.shape[0], 1))
        # print("Direction shape", self.directions.shape)
        self.desired_velocity = np.zeros((self.directions.shape[0], self.points.shape[1]))
        self.desired_accelaration = np.zeros((self.directions.shape[0], self.points.shape[1]))
        self.start_times = np.zeros((self.directions.shape[0], 1))
        self.time = np.zeros((self.directions.shape[0], 1))

        time_each = 0
        for i in range(len(self.points) - 1):
            diff = self.points[i + 1] - self.points[i]
            distance = np.linalg.norm(diff)
            self.distance_segment[i] = distance
            self.directions[i] = diff / distance
            self.desired_velocity[i] = self.v * self.directions[i]

            if i == 0 or i == len(self.points) - 2:
                # if i == 0:

                time_seg = (3 * distance / self.v)
                # time_seg = (distance / self.v)
                scale = np.sqrt(1.65 / time_seg)
                time_seg = time_seg * scale
                self.time[i] = time_seg
                time_each = time_each + time_seg

            else:
                time_seg = distance / self.v
                scale = np.sqrt(1.65 / time_seg)
                time_seg = time_seg * scale
                self.time[i] = time_seg
                time_each = time_each + time_seg

            self.start_times[i] = time_each

        self.start_times = np.vstack((np.zeros(1), self.start_times)).flatten()

        rows = self.points.shape[0]
        columns = 8 * (rows - 1)
        B = lil_matrix((columns, 3))
        A = lil_matrix((columns, columns))
        self.time = self.time.clip(0.25, np.inf)

        for i in range(rows - 1):
            B[8 * i + 3] = self.points[i]
            B[8 * i + 4] = self.points[i + 1]


        counter = 0

        while counter < len(self.time):
            t_each = self.time[counter]
            small = np.array([[0, 0, 0, 0, 0, 0, 0, 1],
                            [t_each ** 7, t_each ** 6, t_each ** 5, t_each ** 4, t_each ** 3, t_each ** 2, t_each, 1],
                            [7 * (t_each ** 6), 6 * (t_each ** 5), 5 * (t_each ** 4), 4 * (t_each ** 3), 3 * (t_each ** 2), 2 * t_each, 1, 0],
                            [42 * (t_each ** 5), 30 * (t_each ** 4), 20 * (t_each ** 3), 12 * (t_each ** 2), 6 * t_each, 2, 0, 0],
                            [210 * (t_each ** 4), 120 * (t_each ** 3), 60 * (t_each ** 2), 24 * t_each, 6, 0, 0, 0],
                            [840 * (t_each ** 3), 360 * (t_each ** 2), 120 * t_each, 24, 0, 0, 0, 0],
                            [2520 * (t_each ** 2), 720 * t_each, 120, 0, 0, 0, 0, 0],
                            [5040 * t_each, 720, 0, 0, 0, 0, 0, 0]])

            A[[0, 1, 2], [6, 5, 4]] = [1, 2, 6]
            if counter != len(self.time) - 1:
                A[5 + 8 * counter, 14 + 8 * counter] = -1
                A[6 + 8 * counter, 13 + 8 * counter] = -2
                A[7 + 8 * counter, 12 + 8 * counter] = -6
                A[8 + 8 * counter, 11 + 8 * counter] = -24
                A[9 + 8 * counter, 10 + 8 * counter] = -120
                A[10 + 8 * counter, 9 + 8 * counter] = -720
                A[8 * counter + 3:8 * counter + 11, 8 * counter:8 * counter + 8] = small
            else:
                A[8 * counter + 3:8 * counter + 11, 8 * counter:8 * counter + 8] = small[:5, :]

            counter += 1

        A = A.tocsc()
        self.C = spsolve(A, B).toarray()



    def update(self, t):
        """
        PRIMARY METHOD
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

        if t >= self.start_times[-1]:
            x_dot = np.zeros((3,))
            x = self.points[-1]

        else:
            ind = np.where(np.sign(self.start_times - t) > 0)
            ind = ind[0][0] - 1
            t = t - self.start_times[ind]
            t_arr_iter = np.array([[t ** 7, t ** 6, t ** 5, t ** 4, t ** 3, t ** 2, t, 1],
                                   [7 * (t ** 6), 6 * (t ** 5), 5 * (t ** 4), 4 * (t ** 3), 3 * (t ** 2), 2 * t, 1, 0],
                                   [42 * (t ** 5), 30 * (t ** 4), 20 * (t ** 3), 12 * (t ** 2), 6 * t, 2, 0, 0],
                                   [210 * (t ** 4), 120 * (t ** 3), 60 * (t ** 2), 24 * t, 6, 0, 0, 0],
                                   [840 * (t ** 3), 360 * (t ** 2), 120 * t, 24, 0, 0, 0, 0]])

            resu = t_arr_iter @ self.C[ind * 8:(ind * 8) + 8, :]
            x = resu[0, :]
            x_dot = resu[1, :]
            x_ddot = resu[2, :]
            x_dddot = resu[3, :]
            x_ddddot = resu[4, :]

        # STUDENT CODE END
        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output




