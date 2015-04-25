#ifndef POINT_H
#define POINT_H

struct Point {
  double x, y;

  Point(double x, double y) {
    this->x = x;
    this->y = y;
  };
};

#endif