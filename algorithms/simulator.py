import sys
import math
import numpy as np
import random

class Simulator():
    X_INIT = 0
    Y_INIT = 0
    A_INIT = 0
    D_INIT = 0

    def __init__(self, X_BOUND, Y_BOUND, ANGLE_BOUND, MEAN_DIST, odom_bias_max=0,
        heading_bias_max=0, distance_bias_max=0):
        self.X_INIT = X_BOUND
        self.Y_INIT = Y_BOUND
        self.A_INIT = ANGLE_BOUND
        self.D_INIT = MEAN_DIST
        self.odom_bias_max = odom_bias_max
        self.heading_bias_max = heading_bias_max
        self.distance_bias_max = distance_bias_max

    def generate_path(self, trace_index, num_points, motion_uncertainties, angle_uncertainties, distance_uncertainties, delta_heading_uncertainties):
        f = open('traces/trace' + str(trace_index), 'w')

        odom_scale1 = 1 + 2 * (random.random() - 0.5) * self.odom_bias_max
        heading_bias1 = 2 * (random.random() - 0.5) * self.heading_bias_max
        distance_scale1 = 1 + 2 * (random.random() - 0.5) * self.distance_bias_max
        odom_scale2 = 1 + 2 * (random.random() - 0.5) * self.odom_bias_max
        heading_bias2 = 2 * (random.random() - 0.5) * self.heading_bias_max
        distance_scale2 = 1 + 2 * (random.random() - 0.5) * self.distance_bias_max

        # Error functions
        def normal(mean, stddev):
            return np.random.normal(mean, stddev)
        def uniform(mean, stddev):
            return np.random.uniform(mean - math.sqrt(3) * stddev, mean + math.sqrt(3) * stddev)

        error_distribution = normal

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
        f.write("H %f %f\n" % (delta_heading_uncertainties[0], delta_heading_uncertainties[1]))
        f.write("\n")

        f.write("# Start input trace \n\n")
        for i in xrange(0, num_points):
            angle_delta1 = random.random() * 2 * self.A_INIT - self.A_INIT
            angle1 = angle_delta1 + 90

            angle_delta2 = random.random() * 2 * self.A_INIT - self.A_INIT
            angle2 = angle_delta2 + 90
            d1 = max(error_distribution(self.D_INIT, self.D_INIT / 4.0), 0.01)
            d2 = max(error_distribution(self.D_INIT, self.D_INIT / 4.0), 0.01)

            gt_x0 = x0 + d1 * math.cos(math.radians(angle1))
            gt_y0 = y0 + d1 * math.sin(math.radians(angle1))

            gt_x1 = x1 + d2 * math.cos(math.radians(angle2))
            gt_y1 = y1 + d2 * math.sin(math.radians(angle2))

            gt_d = math.sqrt((gt_x0 - gt_x1) ** 2 + (gt_y0 - gt_y1) ** 2)

            d1_err = error_distribution(d1 * odom_scale1, math.sqrt(d1 * motion_uncertainties[0]))
            d2_err = error_distribution(d2 * odom_scale2, math.sqrt(d2 * motion_uncertainties[1]))

            angle1_err = error_distribution(angle1 + heading_bias1, math.sqrt(angle_uncertainties[0]))
            angle2_err = error_distribution(angle2 + heading_bias2, math.sqrt(angle_uncertainties[1]))

            pd1_err = error_distribution(gt_d * distance_scale1, math.sqrt(gt_d * distance_uncertainties[0]))
            pd2_err = error_distribution(gt_d * distance_scale2, math.sqrt(gt_d * distance_uncertainties[1]))

            ph1 = math.degrees(math.atan2(gt_y1 - gt_y0, gt_x1 - gt_x0))
            ph2 = math.degrees(math.atan2(gt_y0 - gt_y1, gt_x0 - gt_x1))

            ph1_err = error_distribution(ph1, math.sqrt(delta_heading_uncertainties[0]))
            ph2_err = error_distribution(ph2, math.sqrt(delta_heading_uncertainties[1]))

            f.write("# Point %d\n" % i)
            f.write("M %f %f %f %f\n" % (d1_err, angle1_err, d2_err, angle2_err))
            f.write("# %f %f %f %f\n" % (d1, angle1, d2, angle2))
            f.write("D %f %f %f %f\n" % (pd1_err, pd2_err, ph1_err, ph2_err))
            f.write("# %f %f %f %f\n" % (gt_d, gt_d, ph1, ph2))
            f.write("C %f %f %f %f\n" % (gt_x0, gt_y0, gt_x1, gt_y1))
            f.write("\n")

            x0 = gt_x0
            y0 = gt_y0
            x1 = gt_x1
            y1 = gt_y1

        f.close()
        
if __name__ == "__main__":
    x = Simulator(0, 0, 45, 1)
    if("-h" in sys.argv or len(sys.argv) != 3):
        print "python simulator.py <trace_number> <number of points>"
        sys.exit()
    x.generate_path(int(sys.argv[1]), int(sys.argv[2]), [0.01, 0.01], [0.25, 0.25], [0.04, 0.04], [0.25, 0.25])
