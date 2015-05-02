#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>
#include <boost/python.hpp>
using namespace boost::python;

#include "grid.h"
#include "gridutil.h"
#include "point.h"

class GridPdf {
  Grid *cur_grid;
  Grid *new_grid;
public:
  GridPdf();
  ~GridPdf();
  void initialize(double center_x, double center_y, double delta_x, double delta_y,
    int num_cells_right, int num_cells_up);

  Point get_estimated_location();
  int get_dim_x();
  int get_dim_y();
  double get_probability(int x_idx, int y_idx);

  void move(double distance_moved, double motion_uncertainty, double heading, 
    double heading_uncertainty);
  static void update_distance(GridPdf g1, GridPdf g2, double distance, double distance_uncertainty,
    double heading, double delta_heading_uncertainty);

  void commit();
};

GridPdf::GridPdf() {

}

GridPdf::~GridPdf() {
  // TODO: free memory
}

void GridPdf::initialize(double center_x, double center_y, double delta_x, double delta_y,
    int num_cells_right, int num_cells_up) {
  cur_grid = new Grid(center_x, center_y, delta_x, delta_y, num_cells_right, num_cells_up);
  new_grid = new Grid(center_x, center_y, delta_x, delta_y, num_cells_right, num_cells_up);
}

Point GridPdf::get_estimated_location() {
  return cur_grid->get_estimated_location();
}

int GridPdf::get_dim_x() {
  return cur_grid->num_cells_x;
}

int GridPdf::get_dim_y() {
  return cur_grid->num_cells_y;
}

double GridPdf::get_probability(int x_idx, int y_idx) {
  return cur_grid->pdf[x_idx][y_idx];
}

void GridPdf::move(double distance_moved, double motion_uncertainty, double heading, 
  double heading_uncertainty) {
  new_grid->move(cur_grid, distance_moved, motion_uncertainty, heading, heading_uncertainty);
  commit();
}

void GridPdf::update_distance(GridPdf g1, GridPdf g2, double distance, double distance_uncertainty,
  double heading, double delta_heading_uncertainty) {
  // g1.new_grid->update_distance(g1.cur_grid, g2.cur_grid, distance, distance_uncertainty, 
  //   heading, delta_heading_uncertainty);
  // g1.commit();
  // g2.new_grid->update_distance(g2.cur_grid, g1.cur_grid, distance, distance_uncertainty,
  //   heading + M_PI, delta_heading_uncertainty);
  // g2.commit(); 
}


void GridPdf::commit() {
  new_grid->normalize();
  std::swap(cur_grid, new_grid);
  // cur_grid->recenter(new_grid);
}

BOOST_PYTHON_MODULE(grid_pdf)
{
    class_<GridPdf>("GridPdf")
        .def("initialize", &GridPdf::initialize)
        .def("get_estimated_location", &GridPdf::get_estimated_location)
        .def("get_dim_x", &GridPdf::get_dim_x)
        .def("get_dim_y", &GridPdf::get_dim_y)
        .def("get_probability", &GridPdf::get_probability)
        .def("move", &GridPdf::move)
        .def("update_distance", &GridPdf::update_distance).staticmethod("update_distance");
    class_<Point>("Point")
      .def_readonly("x", &Point::x)
      .def_readonly("y", &Point::y);
};

int main () {
  // For testing
  // gridutil::test();
  GridPdf g1;
  g1.initialize(0,0,0.05,0.05,10,10);
  GridPdf g2;
  g2.initialize(0,0,0.05,0.05,10,10);

  for (int i = 0; i < 1; i++) {
    g1.move(5, 0.5, 0, 0.1);
    g2.move(10, 1, 0, 0.1);
    GridPdf::update_distance(g1, g2, 4.2, 0.5, 0, 0.001);

    Point p = g1.get_estimated_location();
    printf("%lf %lf\n", p.x, p.y);

    p = g2.get_estimated_location();
    printf("%lf %lf\n", p.x, p.y);
  }

  return 0;
}
