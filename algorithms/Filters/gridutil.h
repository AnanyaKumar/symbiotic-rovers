#ifndef GRIDUTIL_H
#define GRIDUTIL_H

#include <cassert>
#include "point.h"

namespace gridutil {
  double distance(Point p1, Point p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
  }

  // Returns an angle between -pi and pi
  double heading(Point p1, Point p2) {
    double x_disp = p2.x - p1.x;
    double y_disp = p2.y - p1.y;
    return atan2(y_disp, x_disp);
  }

  double probability_normal(double mean, double variance, double value) {
    double sigma = sqrt(variance);
    double u = (value - mean) / fabs(sigma);
    double y = (1 / (sqrt(2 * M_PI) * fabs(sigma))) * exp(-u * u / 2);
    return y;
  }

  // Returns the difference between 2 angles, between -pi and pi
  double angle_difference(double angle1, double angle2) {
    double angle_diff = angle2 - angle1;
    if (angle_diff > M_PI) {
      angle_diff -= 2 * M_PI;
    } else if (angle_diff < -M_PI) {
      angle_diff += 2 * M_PI;
    }
    return angle_diff;
  }

  bool almost(double x, double y) {
    double eps = 0.00001;
    return fabs(x - y) < eps;
  }

  void test_distance() {
    Point p1(0, 0);
    Point p2(0.5, 0.5);
    assert(almost(distance(p1, p2), 0.7071067812));

    Point p3(1, 0);
    Point p4(1, 0.5);
    assert(almost(distance(p3, p4), 0.5));

    Point p5(-1, 0.1);
    Point p6(-1, 0.1);
    assert(almost(distance(p5, p6), 0));
  }

  void test_normal() {
    assert(almost(probability_normal(0,1,0), 0.39894228));
  }

  void test_smallest_angle() {
    assert(almost(angle_difference(0.1, 0.2), 0.1));
    assert(almost(angle_difference(3, -3), 0.1));

  }

  void test() {
    test_distance();
    test_normal();
  }
}

#endif