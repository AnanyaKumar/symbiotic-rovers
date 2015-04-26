import math
import sys

from ekf import ExtendedKalmanFilter
from grid_localize import GridLocalize
from odometry_localize import OdometryLocalize

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

class Parser():
    r0_est_pts_x = []
    r0_est_pts_y = []
    r0_gt_pts_x = []
    r0_gt_pts_y = []

    r1_est_pts_x = []
    r1_est_pts_y = []
    r1_gt_pts_x = []
    r1_gt_pts_y = []

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

    def read_trace(self, file, type, verb, plot):
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

            self.r0_est_pts_x = []
            self.r0_est_pts_y = []
            self.r0_gt_pts_x = []
            self.r0_gt_pts_y = []

            self.r1_est_pts_x = []
            self.r1_est_pts_y = []
            self.r1_gt_pts_x = []
            self.r1_gt_pts_y = []

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

                    read_locs = True
                    i = i + 2
                    continue

                if(read_uncertainties < 3):
                    x = s[i].split()
                    if(x[0] == 'O'):
                        m_var = [float(x[1]), float(x[2])]
                    elif(x[0] == 'A'):
                        a_var = [math.radians(float(x[1])), math.radians(float(x[2]))]
                    elif(x[0] == 'D'):
                        d_var = [float(x[1]), float(x[2])]
                    read_uncertainties = read_uncertainties + 1
                    i = i + 1
                    continue

                if(read_num and read_uncertainties == 3 and read_locs):
                    break
            if (type == 0):
                op = OdometryLocalize(init_xy, m_var, a_var, d_var)
            elif(type == 1):
                op = ExtendedKalmanFilter(init_xy, m_var, a_var, d_var)
            elif(type == 2):
                op = GridLocalize(init_xy, m_var, a_var, d_var)
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

                elif x[0] == 'D':
                    d0 = float(x[1])
                    d1 = float(x[2])
                    op.measure_distance([d0, d1])

                elif x[0] == 'C':
                    (x0, y0) = (float(x[1]), float(x[2]))
                    (x1, y1) = (float(x[3]), float(x[4]))
                    (pred_x0, pred_y0) = op.get_pose_estimate(0)
                    (pred_x1, pred_y1) = op.get_pose_estimate(1)
                    if verb:
                        print "Rover 0 abs. pose %d: (%f, %f)" % (counter, x0, y0)
                        print "Rover 0 est. pose %d: (%f, %f)" % (counter, pred_x0, pred_y0)
                    pd0 = math.sqrt((pred_x0 - x0) ** 2 + (pred_y0 - y0) ** 2)
                    d0 = math.sqrt((x0 - static_xy[0][0]) ** 2 + (y0 - static_xy[0][1]) ** 2)
                    self.err0 = (100.0 * (abs(pd0) / d0))
                    if verb:
                        print "Rover 0 error: %f%%" % (100.0 * (abs(pd0) / d0))

                    self.r0_gt_pts_x.extend([x0])
                    self.r0_gt_pts_y.extend([y0])
                    self.r0_est_pts_x.extend([pred_x0])
                    self.r0_est_pts_y.extend([pred_y0])

                    if verb:
                        print "Rover 1 abs. pose %d: (%f, %f)" % (counter, x1, y1)
                        print "Rover 1 est. pose %d: (%f, %f)" % (counter, pred_x1, pred_y1)
                    pd1 = math.sqrt((pred_x1 - x1) ** 2 + (pred_y1 - y1) ** 2)
                    d1 = math.sqrt((x1 - static_xy[1][0]) ** 2 + (y1 - static_xy[1][1]) ** 2)
                    self.err1 = (100.0 * (abs(pd1) / d1))

                    if verb:
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
            self.plot_points(self.r0_gt_pts_x, self.r0_gt_pts_y, self.r0_est_pts_x, self.r0_est_pts_y, self.r1_gt_pts_x, self.r1_gt_pts_y, self.r1_est_pts_x, self.r1_est_pts_y)

if __name__ == "__main__":
    x = Parser()
    if("-h" in sys.argv or len(sys.argv) != 5):
        print "python parser.py <tracefile> <0 = Odom / 1 = EKF / 2 = Grid> <1 = verbose> <1 = plot / 0 = no plot>"
        sys.exit()
    x.read_trace(sys.argv[1], int(sys.argv[2]), bool(sys.argv[3]), bool(sys.argv[4]))
