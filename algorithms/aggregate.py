import sys
import parse
import simulator
import math
import numpy

class Aggregate:
    @staticmethod
    def num_smaller(list1, list2):
        assert len(list1) == len(list2)
        return sum([list1[i] < list2[i] for i in xrange(len(list1))])

    def run(self, num_points, num_iterations):
        x = simulator.Simulator(0, 0, 45, 10)
        for i in xrange(num_iterations):
            x.generate_path(i, num_points,
                            [0.01, 0.01],
                            [25.0, 25.0],
                            [0.04, 0.04],
                            [5.0, 5.0])

        method_names = ["Odometry", "Kalman", "Grid"]
        method_ids = [0, 1, 2]
        control_id = 0
        errors = [[[], []] for i in method_ids]
        combined_errors = [[] for i in method_ids]

        # Collect Errors
        y = parse.Parser()
        for i in xrange(num_iterations):
            for method_id in method_ids:
                y.read_trace("traces/trace" + str(i), method_id, False, False, False)
                errors[method_id][0].append(y.err0)
                errors[method_id][1].append(y.err1)
                combined_errors[method_id].append(y.err0 + y.err1)

        # Process and print errors
        for i in xrange(len(method_ids)):
            error_mean = [sum(errors[i][0]) / num_iterations, sum(errors[i][1]) / num_iterations]
            error_stddev = [numpy.std(errors[i][0]), numpy.std(errors[i][1])]
            error_betters = [Aggregate.num_smaller(errors[i][0], errors[control_id][0]),
                Aggregate.num_smaller(errors[i][1], errors[control_id][1])]
            combined_error_mean = sum(combined_errors[i]) / num_iterations
            combined_error_stddev = numpy.std(combined_errors[i])
            combined_better = Aggregate.num_smaller(combined_errors[i], combined_errors[control_id])

            print "Method %s" % method_names[i]
            print ("-" * 30)
            for j in range(2):
                print ("Rover %d error: %.5f +/- %.5f, %d times better than %s" % 
                    (j, error_mean[j], error_stddev[j] / math.sqrt(num_iterations), 
                    error_betters[j], method_names[control_id]))
            print ("Combined error: %.5f +/- %.5f, %d times better than %s" % 
                (combined_error_mean, combined_error_stddev / math.sqrt(num_iterations), 
                combined_better, method_names[control_id]))
            print "\n"


if __name__ == "__main__":
    x = Aggregate()
    if("-h" in sys.argv or len(sys.argv) != 3):
        print "python aggregate.py <num_points> <num_iterations>"
        sys.exit()
    x.run(int(sys.argv[1]), int(sys.argv[2]))
