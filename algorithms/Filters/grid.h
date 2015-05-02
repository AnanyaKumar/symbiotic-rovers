#ifndef GRID_H
#define GRID_H

#include <utility>

#include "gridutil.h"
#include "point.h"

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

  Grid(double center_x, double center_y, double delta_x, double delta_y,
    int num_cells_right, int num_cells_up);

  Point get_location_from_indices(int xindex, int yindex);
  std::pair<int, int> get_estimated_indices();
  Point get_estimated_location();

  void move(Grid *old_grid, double distance_moved, double motion_uncertainty, double heading, 
    double heading_uncertainty);
  void update_distance(Grid *my_old_grid, Grid *other_old_grid, double distance,
    double distance_uncertainty, double delta_heading, double delta_heading_uncertainty);

  void recenter(Grid *old_grid);
  void normalize();
};

Grid::Grid(double center_x, double center_y, double delta_x, double delta_y,
    int num_cells_right, int num_cells_up) {
  this->center_x = center_x;
  this->center_y = center_y;
  this->delta_x = delta_x;
  this->delta_y = delta_y;
  this->num_cells_right = num_cells_right;
  this->num_cells_up = num_cells_up;
  this->num_cells_x = 2 * num_cells_right + 1;
  this->num_cells_y = 2 * num_cells_up + 1;

  this->pdf = new double*[this->num_cells_x];
  for (int i = 0; i < this->num_cells_x; i++) {
    this->pdf[i] = new double[this->num_cells_y];
    std::fill(this->pdf[i], this->pdf[i] + this->num_cells_y, 0);
  }
  this->pdf[this->num_cells_right][this->num_cells_up] = 1;
}

Point Grid::get_location_from_indices(int xindex, int yindex) {
  Point p(center_x + delta_x * (xindex - num_cells_right),
          center_y + delta_y * (yindex - num_cells_up));
  return p;
}

std::pair<int, int> Grid::get_estimated_indices() {
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
  return std::make_pair(best_xidx, best_yidx);
}

Point Grid::get_estimated_location() {
  std::pair<int, int> best_indices = get_estimated_indices();
  return get_location_from_indices(best_indices.first, best_indices.second);
}

void Grid::move(Grid *old_grid, double distance_moved, double motion_uncertainty, double heading, 
    double heading_uncertainty) {
  center_x = old_grid->center_x + distance_moved * cos(heading);
  center_y = old_grid->center_y + distance_moved * sin(heading);
  for (int xnew_idx = 0; xnew_idx < num_cells_x; xnew_idx++) {
    for (int ynew_idx = 0; ynew_idx < num_cells_y; ynew_idx++) {
      double probability = 0;
      Point new_p = get_location_from_indices(xnew_idx, ynew_idx);
      for (int xold_idx = 0; xold_idx < num_cells_x; xold_idx++) {
        for (int yold_idx = 0; yold_idx < num_cells_y; yold_idx++) {
          Point old_p = old_grid->get_location_from_indices(xold_idx, yold_idx);
          double dist = gridutil::distance(old_p, new_p);
          double dist_probability = gridutil::probability_normal(distance_moved, motion_uncertainty,
              dist);
          double current_heading = gridutil::heading(old_p, new_p);
          double heading_diff = gridutil::angle_difference(current_heading, heading);
          double heading_probability = gridutil::probability_normal(0, heading_uncertainty, 
            heading_diff);
          probability += old_grid->pdf[xold_idx][yold_idx] * dist_probability * heading_probability;
        }
      }
      pdf[xnew_idx][ynew_idx] = probability;
    }
  }
}

void Grid::update_distance(Grid *my_old_grid, Grid *other_old_grid, double distance,
    double distance_uncertainty, double delta_heading, double delta_heading_uncertainty) {
  center_x = my_old_grid->center_x;
  center_y = my_old_grid->center_y;
  for (int xnew_idx = 0; xnew_idx < num_cells_x; xnew_idx++) {
    for (int ynew_idx = 0; ynew_idx < num_cells_y; ynew_idx++) {
      double probability = 0;
      Point new_p = get_location_from_indices(xnew_idx, ynew_idx);
      for (int xother_idx = 0; xother_idx < other_old_grid->num_cells_x; xother_idx++) {
        for (int yother_idx = 0; yother_idx < other_old_grid->num_cells_y; yother_idx++) {
          Point other_p = other_old_grid->get_location_from_indices(xother_idx, yother_idx);
          double dist = gridutil::distance(other_p, new_p);
          double prob_dist_given_data = gridutil::probability_normal(distance, 
            distance_uncertainty, dist);
          double prob_heading_given_data = 1;
          // double cur_heading = gridutil::heading(new_p, other_p);
          // double heading_diff = gridutil::angle_difference(delta_heading, cur_heading);
          // double prob_heading_given_data = gridutil::probability_normal(0, 
          //   delta_heading_uncertainty, heading_diff);
          probability += (prob_dist_given_data * prob_heading_given_data * 
            my_old_grid->pdf[xnew_idx][ynew_idx] * other_old_grid->pdf[xother_idx][yother_idx]);
        }
      }
      pdf[xnew_idx][ynew_idx] = probability;
    }
  }
}

void Grid::recenter(Grid *old_grid) {
  std::pair<int, int> best_indices = old_grid->get_estimated_indices();
  int shift_right = best_indices.first - num_cells_right;
  int shift_up = best_indices.second - num_cells_up;
  Point best_location = old_grid->get_location_from_indices(best_indices.first, best_indices.second);
  center_x = best_location.x;
  center_y = best_location.y;

  for (int x_index = 0; x_index < num_cells_x; x_index++) {
    for (int y_index = 0; y_index < num_cells_y; y_index++) {
      int old_x_index = x_index + shift_right;
      int old_y_index = y_index + shift_up;
      if (old_x_index < 0 || old_x_index >= old_grid->num_cells_x ||
          old_y_index < 0 || old_y_index >= old_grid->num_cells_y) {
        pdf[x_index][y_index] = 0;
      } else {
        pdf[x_index][y_index] = old_grid->pdf[old_x_index][old_y_index];
      }
    }
  }
}

void Grid::normalize() {
  double total_probability = 0;
  for (int i = 0; i < num_cells_x; i++) {
    for (int j = 0; j < num_cells_y; j++) {
      total_probability += pdf[i][j];
    }
  }
  for (int i = 0; i < num_cells_x; i++) {
    for (int j = 0; j < num_cells_y; j++) {
      pdf[i][j] /= total_probability;
    }
  }
}

#endif