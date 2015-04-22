
import math
import sys

from ekf import ExtendedKalmanFilter
from grid_localize import GridLocalize
from odometry_localize import OdometryLocalize

class Parser():
    def read_trace(self, file, type):
        with open(file) as f:
            op = None
            s = f.readlines()

            i = 0
            read_num = False
            read_locs = False
            read_uncertainties = 0
            num = 0

            init_xy = [[0, 0], [0, 0]]
            m_var = [0, 0]
            a_var = [0, 0]
            d_var = [0, 0]

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

                    print "Rover 0 abs. pose %d: (%f, %f)" % (counter, x0, y0)
                    print "Rover 0 est. pose %d: (%f, %f)" % (counter, pred_x0, pred_y0)
                    print "Rover 1 abs. pose %d: (%f, %f)" % (counter, x1, y1)
                    print "Rover 1 est. pose %d: (%f, %f)" % (counter, pred_x1, pred_y1)
                    print "\n"

                    counter = counter + 1
                else:
                    print "Invalid trace file"

                i = i + 1

if __name__ == "__main__":
    x = Parser()
    if(len(sys.argv) != 3):
        print "python parser.py <tracefile> <0 = EKF/1 = GL>"
        sys.exit()
    x.read_trace(sys.argv[1], int(sys.argv[2]))
