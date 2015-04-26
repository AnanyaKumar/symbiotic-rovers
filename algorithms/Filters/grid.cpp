#include <cstdio>
#include <cmath>
#include <algorithm>
#include <boost/python.hpp>
using namespace boost::python;

#include "point.h"
#include "gridutil.h"

struct Grid {
  double delta_x;
  double delta_y;
  int num_cells_right;
  int num_cells_up;
  int num_cells_x;
  int num_cells_y;

  double **pdf;
  double center_x;
  double center_y;

  double **new_pdf;
  double new_center_x;
  double new_center_y;

  Grid();
  ~Grid();
  void initialize(double center_x, double center_y, double delta_x, double delta_y,
    int num_cells_right, int num_cells_up);
  Point get_location_from_indices(int xindex, int yindex);
  Point get_new_location_from_indices(int xindex, int yindex);
  void normalize_new();
  Point get_estimated_location();
  void move(double distance_moved, double motion_uncertainty, double heading, 
    double heading_uncertainty);
  void update_distance(Grid other, double distance, double distance_uncertainty);
  void swap();
};

Grid::Grid() {

}

Grid::~Grid() {
  // TODO: free memory
}

void Grid::initialize(double center_x, double center_y, double delta_x, double delta_y,
    int num_cells_right, int num_cells_up) {
  this->center_x = center_x;
  this->center_y = center_y;
  this->delta_x = delta_x;
  this->delta_y = delta_y;
  this->num_cells_right = num_cells_right;
  this->num_cells_up = num_cells_up;
  this->num_cells_x = 2 * num_cells_right + 1;
  this->num_cells_y = 2 * num_cells_up + 1;
  pdf = new double*[this->num_cells_x];
  new_pdf = new double*[this->num_cells_x];
  for (int i = 0; i < this->num_cells_x; i++) {
    pdf[i] = new double[this->num_cells_y];
    std::fill(pdf[i], pdf[i] + this->num_cells_y, 0);
    new_pdf[i] = new double[this->num_cells_y];
  }
  pdf[num_cells_right][num_cells_up] = 1;
}

Point Grid::get_location_from_indices(int xindex, int yindex) {
  Point p(center_x + delta_x * (xindex - num_cells_right),
          center_y + delta_y * (yindex - num_cells_up));
  return p;
}

Point Grid::get_new_location_from_indices(int xindex, int yindex) {
  Point p(new_center_x + delta_x * (xindex - num_cells_right),
          new_center_y + delta_y * (yindex - num_cells_up));
  return p;
}

Point Grid::get_estimated_location() {
  int best_xidx = 0;
  int best_yidx = 0;
  for (int i = 0; i < num_cells_x; i++) {
    for (int j = 0; j < num_cells_y; j++) {
      if (pdf[i][j] > pdf[best_xidx][best_yidx]) {
        best_xidx = i;
        best_yidx = j;
      }
    }
  }
  return this->get_location_from_indices(best_xidx, best_yidx);
}

void Grid::normalize_new() {
  double total_probability = 0;
  for (int i = 0; i < num_cells_x; i++) {
    for (int j = 0; j < num_cells_y; j++) {
      total_probability += new_pdf[i][j];
    }
  }
  for (int i = 0; i < num_cells_x; i++) {
    for (int j = 0; j < num_cells_y; j++) {
      new_pdf[i][j] /= total_probability;
    }
  }
}

void Grid::move(double distance_moved, double motion_uncertainty, double heading, 
    double heading_uncertainty) {
  new_center_x = center_x + distance_moved * cos(heading);
  new_center_y = center_y + distance_moved * sin(heading);
  for (int xnew_idx = 0; xnew_idx < num_cells_x; xnew_idx++) {
    for (int ynew_idx = 0; ynew_idx < num_cells_y; ynew_idx++) {
      double probability = 0;
      Point new_p = get_new_location_from_indices(xnew_idx, ynew_idx);
      for (int xold_idx = 0; xold_idx < num_cells_x; xold_idx++) {
        for (int yold_idx = 0; yold_idx < num_cells_y; yold_idx++) {
          Point old_p = get_location_from_indices(xold_idx, yold_idx);
          double dist = gridutil::distance(old_p, new_p);
          double dist_probability = gridutil::probability_normal(distance_moved, motion_uncertainty,
              dist);
          double current_heading = gridutil::heading(old_p, new_p);
          double heading_diff = gridutil::angle_difference(current_heading, heading);
          double heading_probability = gridutil::probability_normal(0, heading_uncertainty, 
            heading_diff);
          probability += pdf[xold_idx][yold_idx] * dist_probability * heading_probability;
        }
      }
      new_pdf[xnew_idx][ynew_idx] = probability;
    }
  }
  normalize_new();
}

void Grid::update_distance(Grid other, double distance, double distance_uncertainty) {
  new_center_x = center_x;
  new_center_y = center_y;
  for (int xnew_idx = 0; xnew_idx < num_cells_x; xnew_idx++) {
    for (int ynew_idx = 0; ynew_idx < num_cells_y; ynew_idx++) {
      double probability = 0;
      Point new_p = get_new_location_from_indices(xnew_idx, ynew_idx);
      for (int xother_idx = 0; xother_idx < other.num_cells_x; xother_idx++) {
        for (int yother_idx = 0; yother_idx < other.num_cells_y; yother_idx++) {
          Point other_p = other.get_location_from_indices(xother_idx, yother_idx);
          double dist = gridutil::distance(other_p, new_p);
          double prob_measured_given_data = gridutil::probability_normal(distance, 
            distance_uncertainty, dist);
          probability += (prob_measured_given_data * pdf[xnew_idx][ynew_idx] *
            other.pdf[xother_idx][yother_idx]);
        }
      }
      new_pdf[xnew_idx][ynew_idx] = probability;
    }
  }
  normalize_new();
}

void Grid::swap() {
  std::swap(pdf, new_pdf);
  std::swap(center_x, new_center_x);
  std::swap(center_y, new_center_y);
}

BOOST_PYTHON_MODULE(grid)
{
    class_<Grid>("Grid")
        .def("initialize", &Grid::initialize)
        .def("get_estimated_location", &Grid::get_estimated_location)
        .def("move", &Grid::move)
        .def("update_distance", &Grid::update_distance)
        .def("swap", &Grid::swap);
    class_<Point>("Point")
      .def_readonly("x", &Point::x)
      .def_readonly("y", &Point::y);
};

int main () {
  // For testing
  Grid g;
  g.initialize(0,0,0.05,0.05,20,20);
  gridutil::test();
  for (int i = 0; i < 1; i++) {
    g.move(5, 0.1, 0, 0.1);
    g.swap();
    Point p = g.get_estimated_location();
    printf("%lf %lf\n", p.x, p.y);
  }
  return 0;
}
