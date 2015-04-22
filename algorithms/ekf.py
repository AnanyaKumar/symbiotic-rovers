# Implementation of Extended Kalman Filter
# for symbiotic rover localization problem
#
# Sunny Nahar
# Last updated: 4/21/2015

import numpy as np
import math

class ExtendedKalmanFilter():
    x = []          # Pose vector
    sigma = []      # Pose covariance

    m_var = []      # Motion uncertainty
    a_var = []      # Angle uncertainty
    d_var = []      # Distance uncertainty

    w = []          # Control variances
    u = []          # Control vector

    def __init__(self, start_positions, motion_uncertainties, angle_uncertainties, distance_uncertainties):
        self.x = [start_positions[0][0], start_positions[0][1], start_positions[1][0], start_positions[1][1]]
        self.m_var = motion_uncertainties
        self.a_var = angle_uncertainties
        self.d_var = distance_uncertainties
        self.sigma = [[0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0]]

    def measure_movement(self, distances, directions):
        # Calculate control vector
        self.u = [distances[0], directions[0], distances[1], directions[1]]

        # Calculate control variances
        self.w = [self.m_var[0] * distances[0],
                  self.a_var[0],
                  self.m_var[1] * distances[1],
                  self.a_var[1]]

        # Predicted pose vector
        x_t = np.array([0.0, 0.0, 0.0, 0.0])

        # Update predicted pose vector
        x_t[0] = self.x[0] + (self.u[0]) * math.cos(self.u[1])
        x_t[1] = self.x[1] + (self.u[0]) * math.sin(self.u[1])
        x_t[2] = self.x[2] + (self.u[2]) * math.cos(self.u[3])
        x_t[3] = self.x[3] + (self.u[2]) * math.sin(self.u[3])

        # Predicted pose is new pose
        self.x = x_t

    def measure_distance(self, distances):
        # Covariance noise parameter
        covar_delta = .0000001

        # Measurement covariance matrix
        v = [[self.d_var[0] * distances[0], covar_delta],
             [covar_delta, self.d_var[1] * distances[1]]]

        # Measurement vector
        z = [distances[0], distances[1]]

        # Noise covariance matrix (Assume nonzero between rover
        # heading/odometry)
        Q = np.matrix([[self.w[0] + covar_delta, covar_delta, 0, 0],
                      [covar_delta, self.w[1] + covar_delta, 0, 0],
                      [0, 0, self.w[2] + covar_delta, covar_delta],
                      [0, 0, covar_delta, self.w[3] + covar_delta]])

        # State Jacobian
        F = np.identity(4)

        # Noise Jacobian
        L = np.matrix(
            [[math.cos(self.u[1]), -self.u[0] * math.sin(self.u[1]), 0, 0],
             [math.sin(self.u[1]), self.u[0] * math.cos(self.u[1]), 0, 0],
             [0, 0, math.cos(self.u[3]), -self.u[2] * math.sin(self.u[3])],
             [0, 0, math.sin(self.u[3]), self.u[2] * math.cos(self.u[3])]])

        # Predicted covariance
        sigma_t = np.mat(F) * np.mat(self.sigma) * np.transpose(np.mat(F)) +\
                  np.mat(L) * np.mat(Q) * np.transpose(np.mat(L))

        # Measurement Jacobian
        d = math.sqrt((self.x[0] - self.x[2]) ** 2 +
                      (self.x[1] - self.x[3]) ** 2)
        H_t = np.matrix(
            [[(self.x[0] - self.x[2]) / d, (self.x[1] - self.x[3]) / d,
              (self.x[2] - self.x[0]) / d, (self.x[3] - self.x[1]) / d],
             [(self.x[0] - self.x[2]) / d, (self.x[1] - self.x[3]) / d,
              (self.x[2] - self.x[0]) / d, (self.x[3] - self.x[1]) / d]
             ])

        # Residual covariance
        S_t = np.mat(H_t) * np.mat(sigma_t) * np.transpose(np.mat(H_t)) +\
              np.mat(v)

        #  Kalman gain
        K_t = np.mat(sigma_t) * np.transpose(np.mat(H_t)) *\
              np.linalg.inv(np.mat(S_t))

        # Measurement residual
        dist = math.sqrt((self.x[0] - self.x[2]) ** 2 +
                         (self.x[1] - self.x[3]) ** 2)
        Y_t = np.transpose(np.mat(z)) - np.transpose(np.mat([dist, dist]))

        # Updated state estimate
        x_t_hat = np.transpose(np.mat(self.x)) + np.mat(K_t) * Y_t

        # Updated covariance estimate
        sigma_t_hat = np.mat(np.identity(4) - np.mat(K_t) * np.mat(H_t)) *\
                      np.mat(sigma_t)

        self.x = x_t_hat
        self.sigma = sigma_t_hat

    def get_pose_estimate(self, rover_idx):
        x = np.array(self.x)
        if(rover_idx == 0):
            return [x[0, 0], x[1, 0]]
        elif(rover_idx == 1):
            return [x[2, 0], x[3, 0]]
        else:
            print "fail"

# if __name__ == "__main__":
#     x = ExtendedKalmanFilter([[0, 0], [4, 0]], [0, 0], [0, 0], [0, 0])
#     x.move([3, 6], [math.pi / 2, math.pi / 2])
#     x.measure_distance([5, 5])
#     print x.x

# def update_estimate(x_prev, u_prev, w_prev, Sigma_prev, z_prev, v_prev):
#         # Pose vector:
#         # x = [ x_0,            Rover 0 x
#         #       y_0,            Rover 0 y
#         #       x_1,            Rover 1 x
#         #       y_1 ]           Rover 1 y
#         #
#         # Control state vector:
#         # u = [ d_0,            Rover 0 odometry
#         #       theta_0,        Rover 0 heading
#         #       d_1,            Rover 1 odometry
#         #       theta_1 ]       Rover 1 heading
#         #
#         # Gaussian noise:
#         # w = [ sigma_d0,       Rover 0 odometry variance
#         #       sigma_t0,       Rover 0 heading variance
#         #       sigma_d1,       Rover 1 odometry variance
#         #       sigma_t1 ]      Rover 1 heading variance
#         #
#         # Measurement:
#         # z = [ p ]             Pairwise rover distance
#         #
#         # Measurement noise:
#         # v = [ sigma_p ]       Pairwise noise
#         #
#         # Sigma:                Pose Covariance

#         # small noise
#         noise_delta = .0000000001

#         # Predicted pose vector
#         x_t = np.matrix([[0.0], [0.0], [0.0], [0.0]])

#         # Generate noise (simulation)
#         odom0_noise = np.random.normal(0, math.sqrt(w_prev[0]) + noise_delta)
#         odom1_noise = np.random.normal(0, math.sqrt(w_prev[2]) + noise_delta)
#         head0_noise = np.random.normal(0, math.sqrt(w_prev[1]) + noise_delta)
#         head1_noise = np.random.normal(0, math.sqrt(w_prev[3]) + noise_delta)

#         # Update predicted pose vector
#         x_t[0][0] = x_prev[0] + (u_prev[0] + odom0_noise) * math.cos(u_prev[1] + head0_noise)
#         x_t[1][0] = x_prev[1] + (u_prev[0] + odom0_noise) * math.sin(u_prev[1] + head0_noise)
#         x_t[2][0] = x_prev[2] + (u_prev[2] + odom1_noise) * math.cos(u_prev[3] + head1_noise)
#         x_t[3][0] = x_prev[3] + (u_prev[2] + odom1_noise) * math.sin(u_prev[3] + head1_noise)

#         print "Printing updated pose vector"
#         print np.round(x_t, 4)

#         covar_delta = .0000001

#         # Noise covariance matrix (Assume nonzero between rover heading/odometry)
#         Q = np.matrix([[w_prev[0] + covar_delta, covar_delta, 0, 0],
#                       [covar_delta, w_prev[1] + covar_delta, 0, 0],
#                       [0, 0, w_prev[2] + covar_delta, covar_delta],
#                       [0, 0, covar_delta, w_prev[3] + covar_delta]])

#         print "Noise covariance"
#         print Q

#         # Noise Jacobian
#         L = np.matrix([[math.cos(u_prev[1] + head0_noise), -(u_prev[0] + odom0_noise)
#                       * math.sin(u_prev[1] + head0_noise), 0, 0],
#                        [math.sin(u_prev[1] + head0_noise), (u_prev[0] + odom0_noise)
#                       * math.cos(u_prev[1] + head0_noise), 0, 0],
#                        [0, 0, math.cos(u_prev[3] + head1_noise), -(u_prev[2] + odom1_noise) * math.sin(u_prev[3] + head1_noise)],
#                        [0, 0, math.sin(u_prev[3] + head1_noise), (u_prev[2] + odom1_noise) * math.cos(u_prev[3] + head1_noise)]])

#         print "Noise Jacobian"
#         print L

#         # Predicted covariance
#         Sigma_t = np.mat(Sigma_prev) + np.mat(L) * np.mat(Q) * np.transpose(np.mat(L))
#         print "Predicted Covariance"
#         print np.round(Sigma_t, 5)

#         # Measurement Jacobian
#         d = math.sqrt((x_prev[0] - x_prev[2]) ** 2 + (x_prev[1] - x_prev[3]) ** 2)
#         H_t = np.matrix([[(x_prev[0] - x_prev[2]) / d, (x_prev[1] - x_prev[3]) / d, (x_prev[2] - x_prev[0]) / d, (x_prev[3] - x_prev[1]) / d]])
#         print d
#         print H_t

#         # Residual covariance
#         S_t = np.mat(H_t) * np.mat(Sigma_t) * np.transpose(np.mat(H_t)) + np.mat([[v_prev[0]]])
#         print "Resid covariance"
#         print S_t

#         #  Kalman gain
#         K_t = np.mat(Sigma_t) * np.transpose(np.mat(H_t)) * np.linalg.inv(np.mat(S_t))
#         print "Kalman gain"
#         print K_t

#         # Updated state estimate
#         print "Excess"
#         print np.mat([[z_prev[0] - math.sqrt((x_t[0] - x_t[2]) ** 2 + (x_t[1] - x_t[3]) ** 2)]])
#         x_t_hat = np.mat(x_t) + np.mat(K_t) * np.mat([[z_prev[0] - math.sqrt((x_t[0] - x_t[2]) ** 2 + (x_t[1] - x_t[3]) ** 2)]])

#         # Updated covariance estimate
#         Sigma_t_hat = np.mat(np.identity(4) - np.mat(K_t) * np.mat(H_t)) * np.mat(Sigma_t)

#         return (np.round(x_t_hat, 3), np.round(Sigma_t_hat, 3))