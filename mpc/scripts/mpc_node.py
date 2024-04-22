#!/usr/bin/env python3
import math
from dataclasses import dataclass, field

import cvxpy
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.linalg import block_diag
from scipy.sparse import block_diag, csc_matrix, diags
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
# from utils import nearest_point
from tf_transformations import euler_from_quaternion

# TODO CHECK: include needed ROS msg type headers and libraries
def nearest_point(point, trajectory):
    """
    Return the nearest point along the given piecewise linear trajectory.
    Args:
        point (numpy.ndarray, (2, )): (x, y) of current pose
        trajectory (numpy.ndarray, (N, 2)): array of (x, y) trajectory waypoints
            NOTE: points in trajectory must be unique. If they are not unique, a divide by 0 error will destroy the world
    Returns:
        nearest_point (numpy.ndarray, (2, )): nearest point on the trajectory to the point
        nearest_dist (float): distance to the nearest point
        t (float): nearest point's location as a segment between 0 and 1 on the vector formed by the closest two points on the trajectory. (p_i---*-------p_i+1)
        i (int): index of nearest point in the array of trajectory waypoints
    """
    diffs = trajectory[1:,:] - trajectory[:-1,:]
    l2s   = diffs[:,0]**2 + diffs[:,1]**2
    dots = np.empty((trajectory.shape[0]-1, ))
    for i in range(dots.shape[0]):
        dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])
    t = dots / l2s
    t[t<0.0] = 0.0
    t[t>1.0] = 1.0
    projections = trajectory[:-1,:] + (t*diffs.T).T
    dists = np.empty((projections.shape[0],))
    for i in range(dists.shape[0]):
        temp = point - projections[i]
        dists[i] = np.sqrt(np.sum(temp*temp))
    min_dist_segment = np.argmin(dists)
    return projections[min_dist_segment], dists[min_dist_segment], t[min_dist_segment], min_dist_segment


@dataclass
class mpc_config:
    NXK: int = 4  # length of kinematic state vector: z = [x, y, v, yaw]
    NU: int = 2  # length of input vector: u = = [steering, acceleration]
    TK: int = 10  # finite time horizon length kinematic

    # ---------------------------------------------------
    # TODO: you may need to tune the following matrices
    Rk: list = field(
        default_factory=lambda: np.diag([0.5, 5.50])
    )  # input cost matrix, penalty for inputs - [accel, steering]
    Rdk: list = field(
        default_factory=lambda: np.diag([0.5, 10.50])
    )  # input difference cost matrix, penalty for change of inputs - [accel, steering]
    Qk: list = field(
        # default_factory=lambda: np.diag([13.5, 13.5, 13.0, 5.5])
        default_factory=lambda: np.diag([10.5, 10.5, 8.50, 3.0])
    )  # state error cost matrix, for the the next (T) prediction time steps [x, y, v, yaw]
    Qfk: list = field(
        default_factory=lambda: np.diag([14.0, 14.0, 8.50, 15.0])

    )  # final state error matrix, penalty  for the final state constraints: [x, y, v, yaw]
    # ---------------------------------------------------

    N_IND_SEARCH: int = 20  # Search index number
    DTK: float = 0.170  # time step [s] kinematic
    dlk: float = 0.10  # dist step [m] kinematic
    LENGTH: float = 0.58  # Length of the vehicle [m]
    WIDTH: float = 0.31  # Width of the vehicle [m]
    WB: float = 0.33  # Wheelbase [m]
    MIN_STEER: float = -0.4189  # maximum steering angle [rad]
    MAX_STEER: float = 0.4189  # maximum steering angle [rad]
    MAX_DSTEER: float = np.deg2rad(180.0)  # maximum steering speed [rad/s]
    MAX_SPEED: float = 6.0  # maximum speed [m/s]
    MIN_SPEED: float = 0.0  # minimum backward speed [m/s]
    MAX_ACCEL: float = 4.0  # maximum acceleration [m/ss]

    # # ---------------------------------------------------
    # # TODO: you may need to tune the following matrices
    # Rk: list = field(
    #     default_factory=lambda: np.diag([0.01, 20.0])
    # )  # input cost matrix, penalty for inputs - [accel, steering]
    # Rdk: list = field(
    #     default_factory=lambda: np.diag([7.5, 7.5])
    # )  # input difference cost matrix, penalty for change of inputs - [accel, steering]
    # Qk: list = field(
    #     default_factory=lambda: np.diag([13.5, 13.5, 13.0, 5.5])
    #     # default_factory=lambda: np.diag([20, 20, 15, 100])
    # )  # state error cost matrix, for the the next (T) prediction time steps [x, y, v, yaw]
    # Qfk: list = field(
    #     default_factory=lambda: np.diag([13.5, 13.5, 13.0, 5.5])
    # )  # final state error matrix, penalty  for the final state constraints: [x, y, v, yaw]

@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    v: float = 0.0
    yaw: float = 0.0
    
class MPC(Node):
    """ 
    Implement Kinematic MPC on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('mpc_node')
        # create ROS subscribers and publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10) #publishes message type Ackermann over topic "/drive"
        self.odom_sub = self.create_subscription(Odometry, '/pf/pose/odom', self.pose_callback, 10)

        self.pathpub = self.create_publisher(MarkerArray, '/visualize_path',10)
        # get waypoints
        filename = '/home/moody/f1tenth_ws/waypoints/waypoints.csv'
        # filename = '/home/moody/f1tenth_ws/src/7_MPC/mpc/data/robot_log_with_velocities.csv'
        self.waypoints = np.genfromtxt(filename, delimiter=',')
        self.waypoints[:,3] = 2.0
        # self.waypoints[:,3] = np.abs(self.waypoints[:,3])*2

        # self.waypoints = self.waypoints[np.where(self.waypoints[:,3]>0.01)]

        #trim duplicates
     
        _, unique_indices = np.unique(self.waypoints[:,:2], return_index=True, axis=0)
        self.waypoints = self.waypoints[np.sort(unique_indices),:]


        np.savetxt(filename.replace('.csv', '_corrected.csv'), self.waypoints, delimiter=',', fmt='%f')

        self.config = mpc_config()
        self.odelta = None
        self.oa = None
        self.init_flag = 0

        # initialize MPC problem
        self.mpc_prob_init()
    
    def visualize_path(self, x, y, traj):
        mpc_path = MarkerArray()
        # print(np.shape(x))
        # print(x)
        # print(y)
        for i in range(np.shape(x)[0]):
            path_point = Marker()
            path_point.header.frame_id = "map"
            path_point.ns = "path"
            path_point.id = i
            path_point.type = path_point.SPHERE
            path_point.action = path_point.ADD
            path_point.pose.position.x = x[i]
            path_point.pose.position.y = y[i]
            path_point.pose.position.z = 0.0
            path_point.pose.orientation.x = 0.0
            path_point.pose.orientation.y = 0.0
            path_point.pose.orientation.z = 0.0
            path_point.pose.orientation.w = 1.0
            path_point.scale.x = 0.1
            path_point.scale.y = 0.1
            path_point.scale.z = 0.1
            path_point.color.a = 1.0
            path_point.color.r = 0.0
            path_point.color.g = 0.0
            path_point.color.b = 1.0

            mpc_path.markers.append(path_point)

        waypoints = MarkerArray()
        for i in range(0, np.shape(self.waypoints)[0], 20):
            path_point = Marker()
            path_point.header.frame_id = "map"
            path_point.ns = "path"
            path_point.id = i
            path_point.type = path_point.SPHERE
            path_point.action = path_point.ADD
            path_point.pose.position.x = self.waypoints[i,0]
            path_point.pose.position.y = self.waypoints[i,1]
            path_point.pose.position.z = 0.0
            path_point.pose.orientation.x = 0.0
            path_point.pose.orientation.y = 0.0
            path_point.pose.orientation.z = 0.0
            path_point.pose.orientation.w = 1.0
            path_point.scale.x = 0.2
            path_point.scale.y = 0.2
            path_point.scale.z = 0.2
            path_point.color.a = 1.0
            path_point.color.r = 1.0
            path_point.color.g = 0.0
            path_point.color.b = 0.0

            waypoints.markers.append(path_point)

        # print(np.shape(traj))
        trajectory_mpc = MarkerArray()
        for i in range(np.shape(traj)[1]):
            path_point = Marker()
            path_point.header.frame_id = "map"
            path_point.ns = "path"
            path_point.id = i
            path_point.type = path_point.SPHERE
            path_point.action = path_point.ADD
            path_point.pose.position.x = traj[0,i]
            path_point.pose.position.y = traj[1,i]
            path_point.pose.position.z = 0.0
            path_point.pose.orientation.x = 0.0
            path_point.pose.orientation.y = 0.0
            path_point.pose.orientation.z = 0.0
            path_point.pose.orientation.w = 1.0
            path_point.scale.x = 0.3
            path_point.scale.y = 0.3
            path_point.scale.z = 0.3
            path_point.color.a = 1.0
            path_point.color.r = 0.0
            path_point.color.g = 1.0
            path_point.color.b = 0.0

            trajectory_mpc.markers.append(path_point)
        
        
        self.pathpub.publish(mpc_path)
        self.pathpub.publish(waypoints)
        self.pathpub.publish(trajectory_mpc)

    def pose_callback(self, pose_msg):
      
        # TODO: extract pose from ROS msg
        vehicle_state = State()
        vehicle_state.x = pose_msg.pose.pose.position.x
        vehicle_state.y = pose_msg.pose.pose.position.y
        vehicle_state.v = pose_msg.twist.twist.linear.x

        quaternion = np.array([pose_msg.pose.pose.orientation.x, 
                            pose_msg.pose.pose.orientation.y, 
                            pose_msg.pose.pose.orientation.z, 
                            pose_msg.pose.pose.orientation.w])
        euler = euler_from_quaternion(quaternion)

        vehicle_state.yaw = (euler[2]+(2*math.pi))%(2*math.pi)


        # TODO: Calculate the next reference trajectory for the next T steps
        #       with current vehicle pose.
        #       ref_x, ref_y, ref_yaw, ref_v are columns of self.waypoints
        ref_x = self.waypoints[:,0]
        ref_y = self.waypoints[:,1]
        ref_yaw =(self.waypoints[:,2]+(2*math.pi))%(2*math.pi)
        ref_v = self.waypoints[:,3]
        ref_path = self.calc_ref_trajectory(vehicle_state, ref_x, ref_y, ref_yaw, ref_v) #check
        # print(ref_path)
        x0 = [vehicle_state.x, vehicle_state.y, vehicle_state.v, vehicle_state.yaw]
        # print(np.shape(ref_path))

        # # TODO: solve the MPC control problem
        (
            self.oa,
            self.odelta,
            ox,
            oy,
            oyaw,
            ov,
            state_predict,
        ) = self.linear_mpc_control(ref_path, x0, self.oa, self.odelta)

        # # TODO: publish drive message.
        if self.odelta is not None:
            steer_output = round(self.odelta[0], 3)
            speed_output = round(vehicle_state.v + self.oa[0] * self.config.DTK, 3)
            drive_msg = AckermannDriveStamped()
            # # print(np.shape(state_predict))
            drive_msg.drive.speed = speed_output
            drive_msg.drive.steering_angle = steer_output
            print("- "*20)
            print("speed: " + str(speed_output))
            print("steer: " + str(steer_output))

            self.visualize_path(ox, oy, ref_path)

            # print('--------')
            # print('traj yaw')
            # print(oyaw)
            # print('odelta')
            # print(self.odelta)

            self.drive_pub.publish(drive_msg)

    def mpc_prob_init(self):
        """
        Create MPC quadratic optimization problem using cvxpy, solver: OSQP
        Will be solved every iteration for control.
        More MPC problem information here: https://osqp.org/docs/examples/mpc.html
        More QP example in CVXPY here: https://www.cvxpy.org/examples/basic/quadratic_program.html
        """
        # Initialize and create vectors for the optimization problem
        # Vehicle State Vector
        self.xk = cvxpy.Variable(
            (self.config.NXK, self.config.TK + 1)
        )
        # Control Input vector
        self.uk = cvxpy.Variable(
            (self.config.NU, self.config.TK)
        )
        objective = 0.0  # Objective value of the optimization problem
        constraints = []  # Create constraints array

        # Initialize reference vectors
        self.x0k = cvxpy.Parameter((self.config.NXK,))
        self.x0k.value = np.zeros((self.config.NXK,))

        # Initialize reference trajectory parameter
        self.ref_traj_k = cvxpy.Parameter((self.config.NXK, self.config.TK + 1))
        self.ref_traj_k.value = np.zeros((self.config.NXK, self.config.TK + 1))

        # Initializes block diagonal form of R = [R, R, ..., R] (NU*T, NU*T)
        R_block = block_diag(tuple([self.config.Rk] * self.config.TK))

        # Initializes block diagonal form of Rd = [Rd, ..., Rd] (NU*(T-1), NU*(T-1))
        Rd_block = block_diag(tuple([self.config.Rdk] * (self.config.TK - 1)))

        # Initializes block diagonal form of Q = [Q, Q, ..., Qf] (NX*T, NX*T)
        Q_block = [self.config.Qk] * (self.config.TK)
        Q_block.append(self.config.Qfk)
        Q_block = block_diag(tuple(Q_block))

        # Formulate and create the finite-horizon optimal control problem (objective function)
        # The FTOCP has the horizon of T timesteps

        # --------------------------------------------------------
        # TODO: fill in the objectives here, you should be using cvxpy.quad_form() somehwhere
        objective = 0.0
        # TODO: Objective part 1: Influence of the control inputs: Inputs u multiplied by the penalty R
        objective += cvxpy.quad_form(cvxpy.vec(self.uk), R_block)
        # TODO: Objective part 2: Deviation of the vehicle from the reference trajectory weighted by Q, including final Timestep T weighted by Qf
        objective += cvxpy.quad_form((cvxpy.vec(self.xk) - cvxpy.vec(self.ref_traj_k)),Q_block)
        # TODO: Objective part 3: Difference from one control input to the next control input weighted by Rd
        objective += cvxpy.quad_form((cvxpy.vec(self.uk[:,1:])-cvxpy.vec(self.uk[:,:-1])), Rd_block)
        # --------------------------------------------------------

        # Constraints 1: Calculate the future vehicle behavior/states based on the vehicle dynamics model matrices
        # Evaluate vehicle Dynamics for next T timesteps
        A_block = []
        B_block = []
        C_block = []
        # init path to zeros
        path_predict = np.zeros((self.config.NXK, self.config.TK + 1))
        for t in range(self.config.TK):
            A, B, C = self.get_model_matrix(
                path_predict[2, t], path_predict[3, t], 0.0
            )
     
            A_block.append(A)
            B_block.append(B)
            C_block.extend(C)

        A_block = block_diag(tuple(A_block))
        B_block = block_diag(tuple(B_block))
        C_block = np.array(C_block)

        # [AA] Sparse matrix to CVX parameter for proper stuffing
        # Reference: https://github.com/cvxpy/cvxpy/issues/1159#issuecomment-718925710
        m, n = A_block.shape
      
        self.Annz_k = cvxpy.Parameter(A_block.nnz)
        data = np.ones(self.Annz_k.size)
        rows = A_block.row * n + A_block.col
        cols = np.arange(self.Annz_k.size)
        Indexer = csc_matrix((data, (rows, cols)), shape=(m * n, self.Annz_k.size))

        # Setting sparse matrix data
        self.Annz_k.value = A_block.data

        # Now we use this sparse version instead of the old A_ block matrix
        self.Ak_ = cvxpy.reshape(Indexer @ self.Annz_k, (m, n), order="C")

        # Same as A
        m, n = B_block.shape
        self.Bnnz_k = cvxpy.Parameter(B_block.nnz)
        data = np.ones(self.Bnnz_k.size)
        rows = B_block.row * n + B_block.col
        cols = np.arange(self.Bnnz_k.size)
        Indexer = csc_matrix((data, (rows, cols)), shape=(m * n, self.Bnnz_k.size))
        self.Bk_ = cvxpy.reshape(Indexer @ self.Bnnz_k, (m, n), order="C")
        self.Bnnz_k.value = B_block.data

        # No need for sparse matrices for C as most values are parameters
        self.Ck_ = cvxpy.Parameter(C_block.shape)
        self.Ck_.value = C_block

        # -------------------------------------------------------------
        # TODO: Constraint part 1:
        #       Add dynamics constraints to the optimization problem
        #       This constraint should be based on a few variables:
        #       self.xk, self.Ak_, self.Bk_, self.uk, and self.Ck_
        # print(np.shape(self.Ak_))
        # print(np.shape(self.xk))
        # print(np.shape(self.Bk_))
        # print(np.shape(self.uk))
        # print(np.shape(self.Ck_))

        constraints += [cvxpy.vec(self.xk[:, 1:]) == self.Ak_ @ cvxpy.vec(self.xk[:,:-1]) + self.Bk_ @ cvxpy.vec(self.uk) + self.Ck_]

        # TODO: Constraint part 2:
        #       Add constraints on steering, change in steering angle
        #       cannot exceed steering angle speed limit. Should be based on:
        #       self.uk, self.config.MAX_DSTEER, self.config.DTK
        
        constraints += [self.config.MAX_DSTEER >= cvxpy.abs(self.uk[1, 1:] - self.uk[1, :-1])/self.config.DTK]

        # TODO: Constraint part 3:
        #       Add constraints on upper and lower bounds of states and inputs
        #       and initial state constraint, should be based on:
        #       self.xk, self.x0k, self.config.MAX_SPEED, self.config.MIN_SPEED,
        #       self.uk, self.config.MAX_ACCEL, self.config.MAX_STEER

        constraints += [self.x0k == self.xk[:,0]]
        constraints += [self.config.MIN_STEER <= self.uk[1, :]]
        constraints += [self.config.MAX_STEER >= self.uk[1, :]]
        constraints += [self.config.MIN_SPEED <= self.xk[2,:]]
        constraints += [self.xk[2,:] <= self.config.MAX_SPEED]
        constraints += [cvxpy.abs(self.uk[0,:]) <= self.config.MAX_ACCEL]
        
        # -------------------------------------------------------------

        # Create the optimization problem in CVXPY and setup the workspace
        # Optimization goal: minimize the objective function
        self.MPC_prob = cvxpy.Problem(cvxpy.Minimize(objective), constraints)

    def calc_ref_trajectory(self, state, cx, cy, cyaw, sp):
        """
        calc referent trajectory ref_traj in T steps: [x, y, v, yaw]
        using the current velocity, calc the T points along the reference path
        :param cx: Course X-Position
        :param cy: Course y-Position
        :param cyaw: Course Heading
        :param sp: speed profile
        :dl: distance step
        :pind: Setpoint Index
        :return: reference trajectory ref_traj, reference steering angle
        """

        # Create placeholder Arrays for the reference trajectory for T steps
        ref_traj = np.zeros((self.config.NXK, self.config.TK + 1))
        ncourse = len(cx)

        # Find nearest index/setpoint from where the trajectories are calculated
        _, _, _, ind = nearest_point(np.array([state.x, state.y]), np.array([cx, cy]).T)
        # print(ind)
        # print(cx[ind])
        # print(cy[ind])
        

        # Load the initial parameters from the setpoint into the trajectory
        ref_traj[0, 0] = cx[ind]
        ref_traj[1, 0] = cy[ind]
        ref_traj[2, 0] = sp[ind]
        ref_traj[3, 0] = cyaw[ind]

        # based on current velocity, distance traveled on the ref line between time steps
        travel = abs(state.v) * self.config.DTK
        dind = travel / self.config.dlk
        ind_list = int(ind) + np.insert(
            np.cumsum(np.repeat(dind, self.config.TK)), 0, 0
        ).astype(int)
        ind_list[ind_list >= ncourse] -= ncourse
        ref_traj[0, :] = cx[ind_list]
        ref_traj[1, :] = cy[ind_list]
        ref_traj[2, :] = sp[ind_list]
        cyaw[cyaw - state.yaw > 4.5] = np.abs(
            cyaw[cyaw - state.yaw > 4.5] - (2 * np.pi)
        )
        cyaw[cyaw - state.yaw < -4.5] = np.abs(
            cyaw[cyaw - state.yaw < -4.5] + (2 * np.pi)
        )
        ref_traj[3, :] = cyaw[ind_list]

        #------HARDCODED CHANGE -------
        # ref_traj[0, :] = cx[ind:ind+self.config.TK+1]
        # ref_traj[1, :] = cy[ind:ind+self.config.TK+1]
        # ref_traj[2, :] = sp[ind:ind+self.config.TK+1]
        # ref_traj[3, :] = cyaw[ind:ind+self.config.TK+1]


        return ref_traj

    def predict_motion(self, x0, oa, od, xref):
        path_predict = xref * 0.0
        for i, _ in enumerate(x0):
            path_predict[i, 0] = x0[i]

        state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
        for (ai, di, i) in zip(oa, od, range(1, self.config.TK + 1)):
            state = self.update_state(state, ai, di)
            path_predict[0, i] = state.x
            path_predict[1, i] = state.y
            path_predict[2, i] = state.v
            path_predict[3, i] = state.yaw

        return path_predict

    def update_state(self, state, a, delta):

        # input check
        if delta >= self.config.MAX_STEER:
            delta = self.config.MAX_STEER
        elif delta <= -self.config.MAX_STEER:
            delta = -self.config.MAX_STEER

        state.x = state.x + state.v * math.cos(state.yaw) * self.config.DTK
        state.y = state.y + state.v * math.sin(state.yaw) * self.config.DTK
        state.yaw = (
            state.yaw + (state.v / self.config.WB) * math.tan(delta) * self.config.DTK
        )
        state.v = state.v + a * self.config.DTK

        if state.v > self.config.MAX_SPEED:
            state.v = self.config.MAX_SPEED
        elif state.v < self.config.MIN_SPEED:
            state.v = self.config.MIN_SPEED

        return state

    def get_model_matrix(self, v, phi, delta):
        """
        Calc linear and discrete time dynamic model-> Explicit discrete time-invariant
        Linear System: Xdot = Ax +Bu + C
        State vector: x=[x, y, v, yaw]
        :param v: speed
        :param phi: heading angle of the vehicle
        :param delta: steering angle: delta_bar
        :return: A, B, C
        """

        # State (or system) matrix A, 4x4
        A = np.zeros((self.config.NXK, self.config.NXK))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = self.config.DTK * math.cos(phi)
        A[0, 3] = -self.config.DTK * v * math.sin(phi)
        A[1, 2] = self.config.DTK * math.sin(phi)
        A[1, 3] = self.config.DTK * v * math.cos(phi)
        A[3, 2] = self.config.DTK * math.tan(delta) / self.config.WB

        # Input Matrix B; 4x2
        B = np.zeros((self.config.NXK, self.config.NU))
        B[2, 0] = self.config.DTK
        B[3, 1] = self.config.DTK * v / (self.config.WB * math.cos(delta) ** 2)

        C = np.zeros(self.config.NXK)
        C[0] = self.config.DTK * v * math.sin(phi) * phi
        C[1] = -self.config.DTK * v * math.cos(phi) * phi
        C[3] = -self.config.DTK * v * delta / (self.config.WB * math.cos(delta) ** 2)

        return A, B, C

    def mpc_prob_solve(self, ref_traj, path_predict, x0):
        self.x0k.value = x0

        A_block = []
        B_block = []
        C_block = []
        for t in range(self.config.TK):
            A, B, C = self.get_model_matrix(
                path_predict[2, t], path_predict[3, t], 0.0
            )
            # print(np.shape(A.data))
            A_block.append(A)
            B_block.append(B)
            C_block.extend(C)

        # print(np.shape(A_block))
        A_block = block_diag(tuple(A_block))
        B_block = block_diag(tuple(B_block))
        C_block = np.array(C_block)

        # print(np.shape(A_block))

        # print(np.shape(A_block.data))
        # print(np.shape(B_block.data))
        # print(np.shape(C_block.data))
        self.Annz_k.value = A_block.data
        self.Bnnz_k.value = B_block.data
        self.Ck_.value = C_block

        self.ref_traj_k.value = ref_traj

        # Solve the optimization problem in CVXPY
        # Solver selections: cvxpy.OSQP; cvxpy.GUROBI
        self.MPC_prob.solve(solver=cvxpy.OSQP, verbose=False, warm_start=True)

        if (
            self.MPC_prob.status == cvxpy.OPTIMAL
            or self.MPC_prob.status == cvxpy.OPTIMAL_INACCURATE
        ):
            ox = np.array(self.xk.value[0, :]).flatten()
            oy = np.array(self.xk.value[1, :]).flatten()
            ov = np.array(self.xk.value[2, :]).flatten()
            oyaw = np.array(self.xk.value[3, :]).flatten()
            oa = np.array(self.uk.value[0, :]).flatten()
            odelta = np.array(self.uk.value[1, :]).flatten()

        else:
            print("Error: Cannot solve mpc..")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

        return oa, odelta, ox, oy, oyaw, ov

    def linear_mpc_control(self, ref_path, x0, oa, od):
        """
        MPC contorl with updating operational point iteraitvely
        :param ref_path: reference trajectory in T steps
        :param x0: initial state vector
        :param oa: acceleration of T steps of last time
        :param od: delta of T steps of last time
        """

        if oa is None or od is None:
            oa = [0.0] * self.config.TK
            od = [0.0] * self.config.TK

        # Call the Motion Prediction function: Predict the vehicle motion for x-steps
        path_predict = self.predict_motion(x0, oa, od, ref_path)
        poa, pod = oa[:], od[:]

        # Run the MPC optimization: Create and solve the optimization problem
        mpc_a, mpc_delta, mpc_x, mpc_y, mpc_yaw, mpc_v = self.mpc_prob_solve(
            ref_path, path_predict, x0
        )

        return mpc_a, mpc_delta, mpc_x, mpc_y, mpc_yaw, mpc_v, path_predict

def main(args=None):
    rclpy.init(args=args)
    print("MPC Initialized")
    mpc_node = MPC()
    rclpy.spin(mpc_node)

    mpc_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
