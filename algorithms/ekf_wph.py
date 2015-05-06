# Implementation of Extended Kalman Filter
# for symbiotic rover localization problem
# with pairwise heading
# Sunny Nahar
# Last updated: 4/21/2015

import numpy as np
import math

from localize_interface import Localize_Interface
import util

class ExtendedKalmanFilterWPH(Localize_Interface):
    x = []          # Pose vector
    m_x = []        # Measured pose vector
    sigma = []      # Pose covariance
    m_sigma = []    # Measured covariance
    m_var = []      # Motion uncertainty
    a_var = []      # Angle uncertainty
    d_var = []      # Distance uncertainty
    h_var = []
    w = []          # Control variances
    u = []          # Control vector
    output_x = []
    output_ests = []
    prev_h = []
    @util.overrides(Localize_Interface)
    def __init__(self, start_positions, motion_uncertainties, angle_uncertainties, distance_uncertainties, delta_heading_uncertainties):
        self.x = [start_positions[0][0], start_positions[0][1], start_positions[1][0], start_positions[1][1]]
        self.m_x = [0, 0, 0, 0]
        self.output_x = [0, 0, 0, 0]
        self.output_ests = [[], [], [], []]

        self.m_var = motion_uncertainties
        self.a_var = angle_uncertainties
        self.d_var = distance_uncertainties
        self.h_var = delta_heading_uncertainties

        self.sigma = [[0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0]]

        self.m_sigma = [[0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [0, 0, 0, 0]]
        self.prev_h = [0, 0]

    @util.overrides(Localize_Interface)
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
        self.output_ests = [[x_t[0]], [x_t[1]], [x_t[2]], [x_t[3]]]
        self.output_x = x_t

    @util.overrides(Localize_Interface)
    def measure_distance(self, distances, headings):
        # Covariance noise parameter
        covar_delta = .0000001

        # Measurement covariance matrix
        v = [[self.d_var[0] * distances[0], covar_delta, 0, 0],
             [covar_delta, self.d_var[1] * distances[1], 0, 0],
             [0, 0, self.h_var[0], covar_delta],
             [0, 0, covar_delta, self.h_var[1]]]

        # Measurement vector
        z = [distances[0], distances[1], headings[0], headings[1]]

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
        xdiff = (self.x[0] - self.x[2]) ** 2
        ydiff = (self.x[1] - self.x[3]) ** 2
        H_t = np.matrix(
            [[(self.x[0] - self.x[2]) / d, (self.x[1] - self.x[3]) / d,
              (self.x[2] - self.x[0]) / d, (self.x[3] - self.x[1]) / d],
             [(self.x[0] - self.x[2]) / d, (self.x[1] - self.x[3]) / d,
              (self.x[2] - self.x[0]) / d, (self.x[3] - self.x[1]) / d],
             [(1.0 / d - (xdiff) / (d ** 3)) / math.sqrt(1 - xdiff / (d ** 2)),
              (-1.0 * (self.x[2] - self.x[0]) * (self.x[3] - self.x[1]) / ((d ** 3) * math.sqrt(1 - xdiff / (d ** 2)))),
              (-1.0 * (1.0 / d - (xdiff) / (d ** 3)) / math.sqrt(1 - xdiff / (d ** 2))),
              (1.0 * (self.x[2] - self.x[0]) * (self.x[3] - self.x[1]) / ((d ** 3) * math.sqrt(1 - xdiff / (d ** 2))))],
             [(-1.0 / d + (xdiff) / (d ** 3)) / math.sqrt(1 - xdiff / (d ** 2)),
              (1.0 * (self.x[2] - self.x[0]) * (self.x[3] - self.x[1]) / ((d ** 3) * math.sqrt(1 - xdiff / (d ** 2)))),
              ((1.0 / d - (xdiff) / (d ** 3)) / math.sqrt(1 - xdiff / (d ** 2))),
              (-1.0 * (self.x[2] - self.x[0]) * (self.x[3] - self.x[1]) / ((d ** 3) * math.sqrt(1 - xdiff / (d ** 2))))]
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
        Y_t = np.transpose(np.mat(z)) - np.transpose(np.mat([dist, dist, math.atan2(self.x[3] - self.x[1], self.x[2] - self.x[0]), math.atan2(self.x[1] - self.x[3], self.x[0] - self.x[2])]))

        # Updated state estimate
        x_t_hat = np.transpose(np.mat(self.x)) + np.mat(K_t) * Y_t

        # Updated covariance estimate
        sigma_t_hat = np.mat(np.identity(4) - np.mat(K_t) * np.mat(H_t)) *\
                      np.mat(sigma_t)

        self.x = np.transpose(x_t_hat).tolist()[0]
        self.sigma = sigma_t_hat
        self.output_x = self.x

    @util.overrides(Localize_Interface)
    def get_pose_estimate(self, rover_idx):
        if(rover_idx == 0):
            return [self.output_x[0], self.output_x[1]]
        elif(rover_idx == 1):
            return [self.output_x[2], self.output_x[3]]
        else:
            print "fail"

    def get_possible_pose_estimates(self, rover_idx):
        if(rover_idx == 0):
            return [self.output_ests[0], self.output_ests[1]]
        elif(rover_idx == 1):
            return [self.output_ests[2], self.output_ests[3]]
        else:
            print "fail"
