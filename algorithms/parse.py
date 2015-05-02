import math
import sys

from ekf import ExtendedKalmanFilter
from grid_localize import GridLocalize
from odometry_localize import OdometryLocalize

from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as anim
import time

class Parser():
    NUM_UNCERTAINTIES = 4

    r0_est_pts_x = []
    r0_est_pts_y = []
    r0_gt_pts_x = []
    r0_gt_pts_y = []

    r1_est_pts_x = []
    r1_est_pts_y = []
    r1_gt_pts_x = []
    r1_gt_pts_y = []

    r0_sim_x = []
    r0_sim_y = []

    r1_sim_x = []
    r1_sim_y = []

    err0 = 0
    err1 = 0

    def __init__(self):
        self.err0 = 0
        self.err1 = 0

    def plot_points(self, r0x, r0y, r0xe, r0ye, r1x, r1y, r1xe, r1ye):
        plt.plot(r0x, r0y, 'ob-')
        plt.plot(r0xe, r0ye, 'xc-')

        plt.plot(r1x, r1y, 'or-')
        plt.plot(r1xe, r1ye, 'xm-')

        plt.show()

    def plot_cont(self, xmax, interval, r0x, r0y, r0gtx, r0gty, r1x, r1y, r1gtx, r1gty):
        # print r0x, r0y
        class local:
            x0 = []
            y0 = []
            x1 = []
            y1 = []
            z = []

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        axes = plt.gca()
        plt.ylim((-10, 10))
        # ax.set_xlim([-15, 15])
        # ax.set_ylim([-15, 15])
        # axes.set_zlim([0, 1])
        # ax.set_autoscale_on(False)
        r0gtx[:0] = [0]
        r0gty[:0] = [0]
        r1gtx[:0] = [0]
        r1gty[:0] = [0]
        # ax.set_xlim3d([-10.0, 10.0])

        def update(i):

            plt.axis([0, 6, 0, 20])
            axes.set_xlim([-15, 15])
            axes.set_ylim([-15, 15])
            # axes.set_zlim([0, 1])
            cand_x0 = r0x[2 * i]
            cand_y0 = r0y[2 * i]

            cand_x1 = r1x[2 * i]
            cand_y1 = r1y[2 * i]
            # print local.z
            for j in xrange(0, len(cand_x0)):
                # ax.clear()
                # ax.scatter(r0gtx, r0gty, [0]*len(r0gtx), color='g',marker='o')
                # ax.plot(local.x, local.y, local.z, color='b')
                # ax.scatter(local.x, local.y, local.z, color='b', marker="o")
                # l = len(local.x)
                # ax.plot([local.x[l - 1], cand_x[j]], [local.y[l - 1], cand_y[j]], [0, 0], color='r')
                # ax.scatter([cand_x[j]], [cand_y[j]], [0], color='r', marker="o")
                # fig.canvas.draw()
                # time.sleep(interval / 4000.0)

                fig.clear()
                plt.scatter(r0gtx, r0gty, color='c', marker='o')
                plt.scatter(r1gtx, r1gty, color='m', marker='o')
                plt.plot(local.x0, local.y0, color='b')
                plt.plot(local.x1, local.y1, color='g')
                plt.scatter(local.x0, local.y0, color='b', marker="o")
                plt.scatter(local.x1, local.y1, color='g', marker="o")
                l = len(local.x0)
                plt.plot([local.x0[l - 1], cand_x0[j]], [local.y0[l - 1], cand_y0[j]], color='r')
                plt.scatter([cand_x0[j]], [cand_y0[j]], color='r', marker="o")
                plt.plot([local.x1[l - 1], cand_x1[j]], [local.y1[l - 1], cand_y1[j]], color='r')
                plt.scatter([cand_x1[j]], [cand_y1[j]], color='r', marker="o")
                fig.canvas.draw()
                time.sleep(interval / 1000.0)

            # ax.clear()
            # ax.scatter(r0gtx, r0gty, [0]*len(r0gtx), color='g',marker='o')
            # local.z.append(0)
            # local.x.append(r0x[2 * i + 1])
            # local.y.append(r0y[2 * i + 1])
            # ax.plot(local.x, local.y, local.z, color='b')
            # ax.scatter(local.x, local.y, local.z, color='b', marker="o")

            fig.clear()
            plt.scatter(r0gtx, r0gty, color='c', marker='o')
            plt.scatter(r1gtx, r1gty, color='m', marker='o')
            local.z.append(0)
            local.x0.append(r0x[2 * i + 1])
            local.y0.append(r0y[2 * i + 1])
            local.x1.append(r1x[2 * i + 1])
            local.y1.append(r1y[2 * i + 1])
            plt.plot(local.x0, local.y0, color='b')
            plt.plot(local.x1, local.y1, color='g')
            plt.scatter(local.x0, local.y0, color='b', marker="o")
            plt.scatter(local.x1, local.y1, color='g', marker="o")
        a = anim.FuncAnimation(fig, update, frames=xmax, repeat=False, interval=interval)
        print dir(a)
        plt.show()

    def read_trace(self, file, type, verb, plot, psum):
        with open(file) as f:
            op = None
            s = f.readlines()

            i = 0
            read_num = False
            read_locs = False
            read_uncertainties = 0
            num = 0

            init_xy = [[0, 0], [0, 0]]
            static_xy = [[0, 0], [0, 0]]
            m_var = [0, 0]
            a_var = [0, 0]
            d_var = [0, 0]
            h_var = [0, 0]

            self.r0_est_pts_x = []
            self.r0_est_pts_y = []
            self.r0_gt_pts_x = []
            self.r0_gt_pts_y = []

            self.r1_est_pts_x = []
            self.r1_est_pts_y = []
            self.r1_gt_pts_x = []
            self.r1_gt_pts_y = []


            self.r0_sim_x = []
            self.r0_sim_y = []
            self.r1_sim_x = []
            self.r1_sim_y = []


            while (i < len(s)):
                if(len(s[i]) <= 1 or s[i][0] == '#'):
                    i = i + 1
                    continue

                if(not(read_num)):
                    num = int(s[i].split()[0])
                    read_num = True
                    i = i + 1
                    continue

                if(not(read_locs)):
                    x = s[i].split()
                    x0 = float(x[0])
                    y0 = float(x[1])
                    x = s[i + 1].split()
                    x1 = float(x[0])
                    y1 = float(x[1])
                    init_xy = [[x0, y0], [x1, y1]]
                    static_xy = [[x0, y0], [x1, y1]]

                    self.r0_gt_pts_x.extend([x0])
                    self.r0_gt_pts_y.extend([y0])
                    self.r0_est_pts_x.extend([x0])
                    self.r0_est_pts_y.extend([y0])

                    self.r1_gt_pts_x.extend([x1])
                    self.r1_gt_pts_y.extend([y1])
                    self.r1_est_pts_x.extend([x1])
                    self.r1_est_pts_y.extend([y1])

                    self.r0_sim_x.extend([[], x0])
                    self.r0_sim_y.extend([[], y0])
                    self.r1_sim_x.extend([[], x1])
                    self.r1_sim_y.extend([[], y1])

                    read_locs = True
                    i = i + 2
                    continue

                if(read_uncertainties < self.NUM_UNCERTAINTIES):
                    x = s[i].split()
                    if(x[0] == 'O'):
                        m_var = [float(x[1]), float(x[2])]
                    elif(x[0] == 'A'):
                        a_var = [math.radians(math.sqrt(float(x[1]))) ** 2, math.radians(math.sqrt(float(x[2]))) ** 2]
                    elif(x[0] == 'D'):
                        d_var = [float(x[1]), float(x[2])]
                    elif(x[0] == 'H'):
                        h_var = [float(x[1]), float(x[2])]
                    read_uncertainties = read_uncertainties + 1
                    i = i + 1
                    continue

                if(read_num and read_uncertainties == self.NUM_UNCERTAINTIES and read_locs):
                    break

            if (type == 0):
                op = OdometryLocalize(init_xy, m_var, a_var, d_var, h_var)
            elif(type == 1):
                op = ExtendedKalmanFilter(init_xy, m_var, a_var, d_var, h_var)
            elif(type == 2):
                op = GridLocalize(init_xy, m_var, a_var, d_var, h_var)
            else:
                print "Unknown filter type"
                sys.exit()

            counter = 0
            while(i < len(s)):
                line = s[i]
                if(len(line) <= 1 or line[0] == '#'):
                    i = i + 1
                    continue
                x = line.split()

                if x[0] == 'M':
                    d0 = float(x[1])
                    h0 = math.radians(float(x[2]))

                    d1 = float(x[3])
                    h1 = math.radians(float(x[4]))

                    op.measure_movement([d0, d1], [h0, h1])
                    (x0, y0) = op.get_possible_pose_estimates(0)
                    (x1, y1) = op.get_possible_pose_estimates(1)
                    self.r0_sim_x.extend([x0])
                    self.r0_sim_y.extend([y0])
                    self.r1_sim_x.extend([x1])
                    self.r1_sim_y.extend([y1])

                elif x[0] == 'D':
                    d0 = float(x[1])
                    d1 = float(x[2])
                    ph0 = float(x[3])
                    ph1 = float(x[4])
                    op.measure_distance([d0, d1], [ph0, ph1])
                    (x0, y0) = op.get_pose_estimate(0)
                    (x1, y1) = op.get_pose_estimate(1)
                    self.r0_sim_x.extend([x0])
                    self.r0_sim_y.extend([y0])
                    self.r1_sim_x.extend([x1])
                    self.r1_sim_y.extend([y1])

                elif x[0] == 'C':
                    (x0, y0) = (float(x[1]), float(x[2]))
                    (x1, y1) = (float(x[3]), float(x[4]))
                    (pred_x0, pred_y0) = op.get_pose_estimate(0)
                    (pred_x1, pred_y1) = op.get_pose_estimate(1)
                    if verb or (i == len(s) - 1 and psum):
                        print "Rover 0 abs. pose %d: (%f, %f)" % (counter, x0, y0)
                        print "Rover 0 est. pose %d: (%f, %f)" % (counter, pred_x0, pred_y0)
                    pd0 = math.sqrt((pred_x0 - x0) ** 2 + (pred_y0 - y0) ** 2)
                    d0 = math.sqrt((x0 - static_xy[0][0]) ** 2 + (y0 - static_xy[0][1]) ** 2)
                    self.err0 = (100.0 * (abs(pd0) / d0))
                    if verb or (i == len(s) - 1 and psum):
                        print "Rover 0 error: %f%%" % (100.0 * (abs(pd0) / d0))

                    self.r0_gt_pts_x.extend([x0])
                    self.r0_gt_pts_y.extend([y0])
                    self.r0_est_pts_x.extend([pred_x0])
                    self.r0_est_pts_y.extend([pred_y0])

                    if verb or (i == len(s) - 1 and psum):
                        print "Rover 1 abs. pose %d: (%f, %f)" % (counter, x1, y1)
                        print "Rover 1 est. pose %d: (%f, %f)" % (counter, pred_x1, pred_y1)
                    pd1 = math.sqrt((pred_x1 - x1) ** 2 + (pred_y1 - y1) ** 2)
                    d1 = math.sqrt((x1 - static_xy[1][0]) ** 2 + (y1 - static_xy[1][1]) ** 2)
                    self.err1 = (100.0 * (abs(pd1) / d1))

                    if verb or (i == len(s) - 1 and psum):
                        print "Rover 1 error: %f%%" % (100.0 * (abs(pd1) / d1))
                        print "\n"

                    self.r1_gt_pts_x.extend([x1])
                    self.r1_gt_pts_y.extend([y1])
                    self.r1_est_pts_x.extend([pred_x1])
                    self.r1_est_pts_y.extend([pred_y1])

                    counter = counter + 1
                else:
                    print "Invalid trace file"

                i = i + 1
        if verb:
            print self.r0_gt_pts_x
            print self.r0_gt_pts_y
        if plot:
            self.plot_cont(len(self.r0_sim_x) / 2, 500.0, self.r0_sim_x, self.r0_sim_y, self.r0_gt_pts_x, self.r0_gt_pts_y, self.r1_sim_x, self.r1_sim_y, self.r1_gt_pts_x, self.r1_gt_pts_y)
            # self.plot_points(self.r0_gt_pts_x, self.r0_gt_pts_y, self.r0_est_pts_x, self.r0_est_pts_y, self.r1_gt_pts_x, self.r1_gt_pts_y, self.r1_est_pts_x, self.r1_est_pts_y)


if __name__ == "__main__":
    x = Parser()
    if("-h" in sys.argv or len(sys.argv) != 6):
        print "python parser.py <tracefile> <0 = Odom / 1 = EKF / 2 = Grid> <1 = verbose> <1 = plot / 0 = no plot> <1 = summary>"
        sys.exit()
    x.read_trace(sys.argv[1], int(sys.argv[2]), bool(int(sys.argv[3])), bool(int(sys.argv[4])), bool(int(sys.argv[5])))
