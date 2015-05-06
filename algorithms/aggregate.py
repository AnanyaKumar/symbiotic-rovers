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
        x = simulator.Simulator(0, 0, 45, 5, 0.05, 5.0, 0.05)
        for i in xrange(num_iterations):
            x.generate_path(i, num_points,
                            [0.01, 0.01],
                            [0.0001, 0.0001],
                            [0.1, 0.1],
                            [0.0001, 0.0001])

        method_names = ["Odometry", "Kalman", "Grid", "Kalman w/ H", "UKF", "UKF w/ H", "Particle"]
        method_ids = [0, 1, 3, 4, 5]

        control_id = 0
        errors = [[[], []] for i in method_ids]
        combined_errors = [[] for i in method_ids]
        final_errors = [[[], []] for i in method_ids]
        combined_final_errors = [[] for i in method_ids]

        # Collect Errors
        y = parse.Parser()
        for i in xrange(num_iterations):
            for m in range(len(method_ids)):
                y.read_trace("traces/trace" + str(i), method_ids[m], False, False, False)
                errors[m][0].append(y.err0)
                errors[m][1].append(y.err1)
                combined_errors[m].append(y.err0 + y.err1)
                final_errors[m][0].append(y.err0_final)
                final_errors[m][1].append(y.err1_final)
                combined_final_errors[m].append(y.err0_final + y.err1_final)

        # Process and print errors
        for i in xrange(len(method_ids)):
            print "Method %s" % method_names[i]
            print ("-" * 30)

            def print_stats(tag, errors, combined_errors):
                error_mean = [sum(errors[i][0]) / num_iterations, sum(errors[i][1]) / num_iterations]
                error_stddev = [numpy.std(errors[i][0]), numpy.std(errors[i][1])]
                error_betters = [Aggregate.num_smaller(errors[i][0], errors[control_id][0]),
                    Aggregate.num_smaller(errors[i][1], errors[control_id][1])]
                combined_error_mean = sum(combined_errors[i]) / num_iterations
                combined_error_stddev = numpy.std(combined_errors[i])
                combined_better = Aggregate.num_smaller(combined_errors[i], combined_errors[control_id])

                for j in range(2):
                    print ("Rover %d %s error: %.5f +/- %.5f, %d times better than %s" %
                        (j, tag, error_mean[j], error_stddev[j] / math.sqrt(num_iterations),
                        error_betters[j], method_names[control_id]))
                print ("Combined %s error: %.5f +/- %.5f, %d times better than %s" %
                    (tag, combined_error_mean, combined_error_stddev / math.sqrt(num_iterations),
                    combined_better, method_names[control_id]))
                print ""

            print_stats("Final", final_errors, combined_final_errors)
            print_stats("Accumulated", errors, combined_errors)
            print ""
    def runRandomRestart(self, num_points, num_iterations):
        x = simulator.Simulator(100, 100, 45, 5)
        for i in xrange(num_iterations):
            x.generate_path(i, num_points,
                            [0.0001, 0.1],
                            [0.01, 100],
                            [0.04, 0.4],
                            [0.01, 100])

        method_names = ["Odometry", "Kalman"]
        method_ids = [0, 1]
        control_id = 0
        errors = [[[], []] for i in method_ids]
        combined_errors = [[] for i in method_ids]
        final_errors = [[[], []] for i in method_ids]
        combined_final_errors = [[] for i in method_ids]

        # Collect Errors
        y = parse.Parser()
        for i in xrange(num_iterations):
            for j in xrange(len(method_ids)):
                if method_ids[j] == 0:
                    y.read_trace("traces/trace" + str(i), method_ids[j], False, False, False)
                    errors[j][0].append(y.err0)
                    errors[j][1].append(y.err1)
                    combined_errors[j].append(y.err0 + y.err1)
                    final_errors[j][0].append(y.err0_final)
                    final_errors[j][1].append(y.err1_final)
                    combined_final_errors[j].append(y.err0_final + y.err1_final)
                else:
                    min_err0 = 10 ** 10
                    min_err1 = 10 ** 10
                    min_err0_final = 10 ** 10
                    min_err1_final = 10 ** 10
                    for (k, l) in [(0,0), (100, 0)]:
                            y.read_traceXY("traces/trace" + str(i), method_ids[j], False, False, False, k, l)
                            if(y.err0 + y.err1 < min_err0 + min_err1):
                                min_err0 = y.err0
                                min_err1 = y.err1
                                min_err0_final = y.err0_final
                                min_err1_final = y.err1_final

                    errors[j][0].append(min_err0)
                    errors[j][1].append(min_err1)
                    combined_errors[j].append(min_err0 + min_err1)
                    final_errors[j][0].append(min_err0_final)
                    final_errors[j][1].append(min_err1_final)
                    combined_final_errors[j].append(min_err0_final + min_err1_final)
                    print combined_final_errors[j][i]

        # Process and print errors
        for i in xrange(len(method_ids)):
            print "Method %s" % method_names[i]
            print ("-" * 30)

            def print_stats(tag, errors, combined_errors):
                error_mean = [sum(errors[i][0]) / num_iterations, sum(errors[i][1]) / num_iterations]
                error_stddev = [numpy.std(errors[i][0]), numpy.std(errors[i][1])]
                error_betters = [Aggregate.num_smaller(errors[i][0], errors[control_id][0]),
                    Aggregate.num_smaller(errors[i][1], errors[control_id][1])]
                combined_error_mean = sum(combined_errors[i]) / num_iterations
                combined_error_stddev = numpy.std(combined_errors[i])
                combined_better = Aggregate.num_smaller(combined_errors[i], combined_errors[control_id])

                for j in range(2):
                    print ("Rover %d %s error: %.5f +/- %.5f, %d times better than %s" %
                        (j, tag, error_mean[j], error_stddev[j] / math.sqrt(num_iterations),
                        error_betters[j], method_names[control_id]))
                print ("Combined %s error: %.5f +/- %.5f, %d times better than %s" %
                    (tag, combined_error_mean, combined_error_stddev / math.sqrt(num_iterations),
                    combined_better, method_names[control_id]))
                print ""

            print_stats("Final", final_errors, combined_final_errors)
            print_stats("Accumulated", errors, combined_errors)
            print ""

if __name__ == "__main__":
    x = Aggregate()
    if("-h" in sys.argv or len(sys.argv) != 3):
        print "python aggregate.py <num_points> <num_iterations>"
        sys.exit()
    x.run(int(sys.argv[1]), int(sys.argv[2]))
