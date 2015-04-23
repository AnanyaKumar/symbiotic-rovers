import sys
import math
import numpy as np
import random

class Simulator():
    X_INIT = 0
    Y_INIT = 0
    A_INIT = 0
    D_INIT = 0

    def __init__(self, X_BOUND, Y_BOUND, ANGLE_BOUND, MEAN_DIST):
        self.X_INIT = X_BOUND
        self.Y_INIT = Y_BOUND
        self.A_INIT = ANGLE_BOUND
        self.D_INIT = MEAN_DIST

    def generate_path(self, trace_index, num_points, motion_uncertainties, angle_uncertainties, distance_uncertainties):
        f = open('traces/trace' + str(trace_index), 'w')

        f.write("%d # Number of rovers\n\n" % 2)

        x0 = random.random() * self.X_INIT  # 2 * self.X_INIT - self.X_INIT
        y0 = random.random() * self.Y_INIT  # 2 * self.Y_INIT - self.Y_INIT
        x1 = random.random() * self.X_INIT  # 2 * self.X_INIT - self.X_INIT
        y1 = random.random() * self.Y_INIT  # 2 * self.Y_INIT - self.Y_INIT

        f.write("# Initial rover locations\n")
        f.write("%f %f \n" % (x0, y0))
        f.write("%f %f \n" % (x1, y1))
        f.write("\n")

        f.write("# Uncertainties\n")
        f.write("O %f %f\n" %
                (motion_uncertainties[0], motion_uncertainties[1]))
        f.write("A %f %f\n" % (angle_uncertainties[0], angle_uncertainties[1]))
        f.write("D %f %f\n" % (distance_uncertainties[0], distance_uncertainties[1]))
        f.write("\n")

        f.write("# Start input trace \n\n")
        for i in xrange(0, num_points):
            angle_delta1 = random.random() * 2 * self.A_INIT - self.A_INIT
            angle1 = angle_delta1 + 90

            angle_delta2 = random.random() * 2 * self.A_INIT - self.A_INIT
            angle2 = angle_delta2 + 90

            d1 = max(np.random.normal(self.D_INIT, self.D_INIT / 4), 0.01)
            d2 = max(np.random.normal(self.D_INIT, self.D_INIT / 4), 0.01)

            gt_x0 = x0 + d1 * math.cos(math.radians(angle1))
            gt_y0 = y0 + d1 * math.sin(math.radians(angle1))

            gt_x1 = x1 + d2 * math.cos(math.radians(angle2))
            gt_y1 = y1 + d2 * math.sin(math.radians(angle2))

            gt_d = math.sqrt((gt_x0 - gt_x1) ** 2 + (gt_y0 - gt_y1) ** 2)

            d1_err = np.random.normal(d1, d1 * motion_uncertainties[0])
            d2_err = np.random.normal(d2, d2 * motion_uncertainties[1])

            angle1_err = np.random.normal(angle1, angle_uncertainties[0])
            angle2_err = np.random.normal(angle2, angle_uncertainties[1])

            pd1_err = np.random.normal(gt_d, gt_d * distance_uncertainties[0])
            pd2_err = np.random.normal(gt_d, gt_d * distance_uncertainties[1])

            f.write("M %f %f %f %f\n" % (d1_err, angle1_err, d2_err, angle2_err))
            f.write("# %f %f %f %f\n" % (d1, angle1, d2, angle2))
            f.write("D %f %f\n" % (pd1_err, pd2_err))
            f.write("C %f %f %f %f\n" % (gt_x0, gt_y0, gt_x1, gt_y1))

            x0 = gt_x0
            y0 = gt_y0
            x1 = gt_x1
            y1 = gt_y1

        f.close()
if __name__ == "__main__":
    x = Simulator(10, 10, 90, 15)
    if("-h" in sys.argv or len(sys.argv) != 3):
        print "python simulator.py <trace_number> <number of points>"
        sys.exit()
    x.generate_path(int(sys.argv[1]), int(sys.argv[2]), [0.02, 0.02], [10.0, 10.0], [0.01, 0.01])
