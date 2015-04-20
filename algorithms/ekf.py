import numpy as np
import math

def update_estimate(x_tm1, u_tm1, w_tm1, Sigma_tm1, z_tm1, v_tm1):
    # Pose vector:
    # x = [ x_0,            Rover 0 x
    #       y_0,            Rover 0 y
    #       x_1,            Rover 1 x
    #       y_1 ]           Rover 1 y
    #
    # Control state vector:
    # u = [ d_0,            Rover 0 odometry
    #       theta_0,        Rover 0 heading
    #       d_1,            Rover 1 odometry
    #       theta_1 ]       Rover 1 heading
    #
    # Gaussian noise:
    # u = [ sigma_d0,       Rover 0 odometry variance
    #       sigma_t0,       Rover 0 heading variance
    #       sigma_d1,       Rover 1 odometry variance
    #       sigma_t1 ]      Rover 1 heading variance
    #
    # Measurement:
    # z = [ p ]             Pairwise rover distance
    #
    # Measurement noise:
    # v = [ sigma_p ]       Pairwise noise

    # Predicted pose vector
    x_t = np.matrix([[0], [0], [0], [0]])

    # Generate noise (simulation)
    odom0_noise = np.random.normal(0, math.sqrt(w_tm1[0]))
    odom1_noise = np.random.normal(0, math.sqrt(w_tm1[2]))
    head0_noise = np.random.normal(0, math.sqrt(w_tm1[1]))
    head1_noise = np.random.normal(0, math.sqrt(w_tm1[3]))

    # Update predicted pose vector
    x_t[0][0] = x_tm1[0] + (u_tm1[0] + odom0_noise) * math.cos(u_tm1[1] + head0_noise)
    x_t[1][0] = x_tm1[1] + (u_tm1[0] + odom0_noise) * math.sin(u_tm1[1] + head0_noise)
    x_t[2][0] = x_tm1[2] + (u_tm1[2] + odom1_noise) * math.cos(u_tm1[3] + head1_noise)
    x_t[3][0] = x_tm1[3] + (u_tm1[2] + odom1_noise) * math.sin(u_tm1[3] + head1_noise)

    nz = .00001

    # Noise covariance matrix (Assume nonzero between rover heading/odometry)
    Q = np.matrix([[w_tm1[0], nz, 0, 0],
                  [nz, w_tm1[1], 0, 0],
                  [0, 0, w_tm1[2], nz],
                  [0, 0, nz, w_tm1[3]]])

    # Noise Jacobian
    L = np.matrix([[math.cos(u_tm1[1] + head0_noise), -(u_tm1[0] + odom0_noise) * math.sin(u_tm1[1] + head0_noise), 0, 0],
                  [math.sin(u_tm1[1] + head0_noise), (u_tm1[0] + odom0_noise) * math.cos(u_tm1[1] + head0_noise), 0, 0],
                  [0, 0, math.cos(u_tm1[3] + head1_noise), -(u_tm1[2] + odom1_noise) * math.sin(u_tm1[3] + head1_noise)],
                  [0, 0, math.sin(u_tm1[3] + head1_noise), (u_tm1[2] + odom1_noise) * math.cos(u_tm1[3] + head1_noise)]])

    # Predicted covariance
    Sigma_t = np.mat(Sigma_tm1) + np.mat(L) * np.mat(Q) * np.transpose(np.mat(L))

    # Measurement Jacobian
    d = math.sqrt((x_tm1[0] - x_tm1[2]) ** 2 + (x_tm1[1] - x_tm1[3]) ** 2)
    H_t = np.matrix([[(x_tm1[0] - x_tm1[2]) / d, (x_tm1[1] - x_tm1[3]) / d, (x_tm1[2] - x_tm1[0]) / d, (x_tm1[3] - x_tm1[1]) / d]])

    # Residual covariance
    S_t = np.mat(H_t) * np.mat(Sigma_t) * np.transpose(np.mat(H_t)) + np.mat([[v_tm1[0]]])

    #  Kalman gain
    K_t = np.mat(Sigma_t) * np.transpose(np.mat(H_t)) * np.linalg.inv(np.mat(S_t))

    # Updated state estimate
    x_t_hat = np.mat(x_t) + np.mat(K_t) * np.mat([[z_tm1[0]]])

    # Updated covariance estimate
    Sigma_t_hat = np.mat(np.identity(4) - np.mat(K_t) * np.mat(H_t)) * np.mat(Sigma_t)

    return (x_t_hat, Sigma_t_hat)