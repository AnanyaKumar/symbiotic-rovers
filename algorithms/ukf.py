# Implementation of Unscented Kalman Filter
# for symbiotic rover localization problem
#
# Sunny Nahar
# Last updated: 5/02/2015

import numpy as np
import math
import scipy.linalg as sp
from localize_interface import Localize_Interface
import util

class UnscentedKalmanFilter(Localize_Interface):
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
    f_pts = []

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

        covar_delta = 0.000000001
        self.sigma = [[covar_delta, 0, 0, 0],
                      [0, covar_delta, 0, 0],
                      [0, 0, covar_delta, 0],
                      [0, 0, 0, covar_delta]]
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

        # Predicted pose vector (x_{k-1,k-1})
        X = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        X[0] = self.x[0]
        X[1] = self.x[1]
        X[2] = self.x[2]
        X[3] = self.x[3]

        s = np.matrix(self.sigma)
        s = (np.matrix(s) + np.transpose(np.matrix(s))) / 2

        covar_delta = .0000001

        # P_{k-1,k-1}
        P = np.matrix([[s[0, 0], s[0, 1], s[0, 2], s[0, 3], 0, 0, 0, 0],
                      [s[1, 0], s[1, 1], s[1, 2], s[1, 3], 0, 0, 0, 0],
                      [s[2, 0], s[2, 1], s[2, 2], s[2, 3], 0, 0, 0, 0],
                      [s[3, 0], s[3, 1], s[3, 2], s[3, 3], 0, 0, 0, 0],
                      [0, 0, 0, 0, self.w[0] + covar_delta, covar_delta, 0, 0],
                      [0, 0, 0, 0, covar_delta, self.w[1] + covar_delta, 0, 0],
                      [0, 0, 0, 0, 0, 0, self.w[2] + covar_delta, covar_delta],
                      [0, 0, 0, 0, 0, 0, covar_delta, self.w[3] + covar_delta]])
        alpha = 0.0001
        kappa = 0
        beta = 2

        L = 8.0
        lam = alpha ** 2 * (L + kappa) - L

        SQ = sp.sqrtm(np.matrix((L + lam) * P))
        SQ = np.real(SQ)

        pts = [X,
               X + SQ[:, 0],
               X + SQ[:, 1],
               X + SQ[:, 2],
               X + SQ[:, 3],
               X + SQ[:, 4],
               X + SQ[:, 5],
               X + SQ[:, 6],
               X + SQ[:, 7],
               X - SQ[:, 0],
               X - SQ[:, 1],
               X - SQ[:, 2],
               X - SQ[:, 3],
               X - SQ[:, 4],
               X - SQ[:, 5],
               X - SQ[:, 6],
               X - SQ[:, 7]]
        # print pts
        # Update predicted pose vector
        f_pts = []
        for i in xrange(0, 17):
            f_pts.append([(pts[i])[0] + (self.u[0] + (pts[i])[4]) * math.cos(
                          self.u[1] + (pts[i])[5]),
                          (pts[i])[1] + (self.u[0] + (pts[i])[4]) * math.sin(
                          self.u[1] + (pts[i])[5]),
                          (pts[i])[2] + (self.u[2] + (pts[i])[6]) * math.cos(
                          self.u[3] + (pts[i])[7]),
                          (pts[i])[3] + (self.u[2] + (pts[i])[6]) * math.sin(
                          self.u[3] + (pts[i])[7])])

        # print f_pts
        w_sum = np.array([0, 0, 0, 0])
        for i in xrange(1, 17):
            w_sum = w_sum + np.array(f_pts[i]) * 1.0 / (2.0 * (L + lam))
        w_sum = w_sum + np.array(f_pts[0]) * (lam / (L + lam))

        p_sum = np.matrix([[0, 0, 0, 0],
                          [0, 0, 0, 0],
                          [0, 0, 0, 0],
                          [0, 0, 0, 0]])
        for i in xrange(1, 17):
            p_sum = p_sum + ((np.transpose(np.matrix(np.array(f_pts[i]) -
                             np.array(w_sum))) * (np.matrix(np.array(f_pts[i])
                             - np.array(w_sum))))) * 1.0 / (2.0 * (L + lam))
        p_sum = p_sum + ((np.transpose(np.matrix(np.array(f_pts[0]) -
                np.array(w_sum))) * (np.matrix(np.array(f_pts[0]) -
                np.array(w_sum))))) * (lam / (L + lam) + (1 - alpha ** 2 + beta
                ))

        # Predicted pose is new pose
        self.m_x = w_sum
        self.m_sigma = p_sum
        self.f_pts = np.array(f_pts)

    @util.overrides(Localize_Interface)
    def measure_distance(self, distances, headings):
        h0 = headings[0]
        h1 = headings[1]
        if(h0 - h1 < 0):
            diff = math.pi - abs(h0 - h1)
            if(h1 - h0 > math.pi):
                h1 = h1 - diff / 2.0
                h0 = h0 + diff / 2.0
            else:
                h1 = h1 + diff / 2.0
                h0 = h0 - diff / 2.0
        else:
            diff = math.pi - abs(h1 - h0)
            if(h1 - h0 > math.pi):
                h1 = h1 + diff / 2.0
                h0 = h0 - diff / 2.0
            else:
                h1 = h1 - diff / 2.0
                h0 = h0 + diff / 2.0

        z_k = np.array([distances[0], distances[1]])

        X = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        X[0] = self.m_x[0]
        X[1] = self.m_x[1]
        X[2] = self.m_x[2]
        X[3] = self.m_x[3]

        s = np.matrix(self.m_sigma)
        s = (np.matrix(s) + np.transpose(np.matrix(s))) / 2
        covar_delta = .0000001

        # P_{k-1,k-1}
        P = np.matrix([[s[0, 0], s[0, 1], s[0, 2], s[0, 3], 0, 0],
                      [s[1, 0], s[1, 1], s[1, 2], s[1, 3], 0, 0],
                      [s[2, 0], s[2, 1], s[2, 2], s[2, 3], 0, 0],
                      [s[3, 0], s[3, 1], s[3, 2], s[3, 3], 0, 0],
                      [0, 0, 0, 0, self.d_var[0] * distances[0], covar_delta],
                      [0, 0, 0, 0, covar_delta, self.d_var[1] * distances[1]]])

        alpha = .0001
        kappa = 0
        beta = 2

        L = 6.0
        lam = alpha ** 2 * (L + kappa) - L

        SQ = sp.sqrtm(np.matrix((L + lam) * P))
        SQ = np.real(SQ)

        pts = [X,
               X + SQ[:, 0],
               X + SQ[:, 1],
               X + SQ[:, 2],
               X + SQ[:, 3],
               X + SQ[:, 4],
               X + SQ[:, 5],
               X - SQ[:, 0],
               X - SQ[:, 1],
               X - SQ[:, 2],
               X - SQ[:, 3],
               X - SQ[:, 4],
               X - SQ[:, 5]]

        # Update predicted pose vector
        f_pts = []
        for i in xrange(0, 13):
            p = pts[i]
            d = math.sqrt((p[2] - p[0]) ** 2 + (p[2] - p[0]) ** 2)
            f_pts.append([d + p[4],
                          d + p[5]])

        # z_k hat
        z_khat = np.array([ 0, 0])
        for i in xrange(1, 13):
            z_khat = z_khat + np.array(f_pts[i]) * 1.0 / (2.0 * (L + lam))
        z_khat = z_khat + np.array(f_pts[0]) * (lam / (L + lam))

        P_ZkZk = np.matrix([[0, 0],
                           [0, 0]])

        # P_{zk,zk}
        for i in xrange(1, 13):
            P_ZkZk = P_ZkZk + ((np.transpose(np.matrix(np.array(f_pts[i]) - np.array(z_khat))) * (np.matrix(np.array(f_pts[i]) - np.array(z_khat))))) * 1.0 / (2.0 * (L + lam))
        P_ZkZk = P_ZkZk + ((np.transpose(np.matrix(np.array(f_pts[0]) - np.array(z_khat))) * (np.matrix(np.array(f_pts[0]) - np.array(z_khat))))) * (lam / (L + lam) + (1 - alpha ** 2 + beta))

        # P_{xk,zk}
        P_XkZk = np.matrix([[0, 0],
                           [0, 0],
                           [0, 0],
                           [0, 0]])
        for i in xrange(1, 13):
            P_XkZk = P_XkZk + ((np.transpose(np.matrix(np.array(self.f_pts[i]) - np.array(self.m_x))) * (np.matrix(np.array(f_pts[i]) - np.array(z_khat))))) * 1.0 / (2.0 * (L + lam))
        P_XkZk = P_XkZk + ((np.transpose(np.matrix(np.array(self.f_pts[0]) - np.array(self.m_x))) * (np.matrix(np.array(f_pts[0]) - np.array(z_khat))))) * (lam / (L + lam) + (1 - alpha ** 2 + beta))

        # Kalman gain
        K = np.matrix(P_XkZk) * np.linalg.inv(np.matrix(P_ZkZk))

        # x_k|k
        x_kk = np.transpose(np.transpose(np.matrix(self.m_x)) + np.matrix(K) *
               np.transpose(np.matrix((z_k - z_khat))))

        # P_kk
        P_kk = np.matrix(self.m_sigma) - np.matrix(K) * np.matrix(P_ZkZk) * np.transpose(np.matrix(K))

        self.x = np.array(x_kk)[0]

        self.sigma = P_kk
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
