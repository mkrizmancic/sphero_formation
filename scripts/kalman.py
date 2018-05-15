import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class KalmanFilter():
    def __init__(self, position, velocity):
        self.step_time = 0.1
        self.X0 = self.get_numpy_state(position, velocity)
        self.P0 = 0.001 * np.eye(4)  # Ne znam kako izgleda pocetna matrica
        self.Q = 0.1 * np.eye(4)
        self.R = 0.001 * np.eye(2)

        T = self.step_time
        self.A = np.array([[1, 0, T, 0], [0, 1, 0, T], [0, 0, 1, 0], [0, 0, 0, 1]])
        self.B = np.array([[]])
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])

        self.X_old = self.X0
        self.P_old = self.P0

    def get_numpy_state(self, position, velocity=Twist()):
        """Convert from some type (here: ROS msg) to numpy array."""
        x = position.position.x
        y = position.position.y
        vx = velocity.linear.x
        vy = velocity.linear.y
        state = np.array([[x, y, vx, vy]])
        return state.T

    def get_used_state(self, np_state):
        """Convert from numpy array to type used elswhere (here: ROS msg)."""
        time = rospy.Time.now()
        msg = Odometry()
        msg.header.stamp = time
        msg.pose.pose.position.x = np_state[0][0]
        msg.pose.pose.position.y = np_state[1][0]
        msg.twist.twist.linear.x = np_state[2][0]
        msg.twist.twist.linear.y = np_state[3][0]
        return msg

    def predict(self, u):
        """
        Args:
            u: input vector
        """
        X_est = np.dot(self.A, self.X_old)
        P_est = np.dot(np.dot(self.A, self.P_old), self.A.T) + self.Q

        self.X_old = X_est
        self.P_old = P_est

        return X_est, P_est

    def update(self, X_est, P_est, Xm):
        """
        Args:
            Xm: measured state
            X_est: estimated state from prediction step
            P_est: estimated covariance matrix from prediction step
        """
        Xm = self.get_numpy_state(Xm)
        K = np.dot(np.dot(P_est, self.H.T), np.linalg.inv(np.dot(np.dot(self.H, P_est), self.H.T) + self.R))
        Y = np.dot(self.H, Xm)
        X_new = X_est + np.dot(K, (Y - np.dot(self.H, X_est)))
        P_new = np.eye(4) - np.dot(np.dot(K, self.H), P_est)

        self.X_old = X_new
        self.P_old = P_new

        return self.get_used_state(X_new)
