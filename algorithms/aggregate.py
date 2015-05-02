import sys
import parse
import simulator
import math

class Aggregate:
    def run(self, num_points, num_iterations):
        x = simulator.Simulator(0, 0, 45, 1)
        for i in xrange(num_iterations):
            x.generate_path(i, num_points,
                            [0.000001, 0.05],
                            [0.000001, 1.0],
                            [0.000001, 0.1],
                            [0.000001, 1.0])
        o_sum_0 = 0
        ekf_sum_0 = 0
        grid_sum_0 = 0
        o_sum_1 = 0
        ekf_sum_1 = 0
        grid_sum_1 = 0
        y = parse.Parser()
        for i in xrange(num_iterations):
            y.read_trace("traces/trace" + str(i), 0, False, False, False)
            o_sum_0 = o_sum_0 + y.err0
            o_sum_1 = o_sum_1 + y.err1

            y.read_trace("traces/trace" + str(i), 1, False, False, False)
            ekf_sum_0 = ekf_sum_0 + y.err0
            ekf_sum_1 = ekf_sum_1 + y.err1

            # y.read_trace("traces/trace" + str(i), 2, False, False)
            # grid_sum_0 = grid_sum_0 + y.err0
            # grid_sum_1 = grid_sum_1 + y.err1

        print "Odometry error: Rover 0:%f Rover 1:%f" % (o_sum_0 / num_iterations, o_sum_1 / num_iterations)
        print "EKF error:      Rover 0:%f Rover 1:%f   %d times better" % (ekf_sum_0 / num_iterations, ekf_sum_1 / num_iterations, int(math.floor(o_sum_1 / ekf_sum_1)))
        print "Grid error:     Rover 0:%f Rover 1:%f" % (grid_sum_0 / num_iterations, grid_sum_1 / num_iterations)

if __name__ == "__main__":
    x = Aggregate()
    if("-h" in sys.argv or len(sys.argv) != 3):
        print "python aggregate.py <num_points> <num_iterations>"
        sys.exit()
    x.run(int(sys.argv[1]), int(sys.argv[2]))
