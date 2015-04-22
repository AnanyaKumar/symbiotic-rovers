
import math

from localize_interface import Localize_Interface
import util

class Grid:

  center_x = 0
  center_y = 0
  delta_x = 0.1
  delta_y = 0.1
  num_cells_right = 10
  num_cells_up = 10
  pdf = [[]]

  def __init__(self, center_x=0, center_y=0, delta_x=0.1, delta_y=0.1, num_cells_right=10,
    num_cells_up=10):
    self.center_x = center_x
    self.center_y = center_y
    self.delta_x = delta_x
    self.delta_y = delta_y
    self.num_cells_right = num_cells_right
    self.num_cells_up = num_cells_up
    self.pdf = [[0 for i in range(2 * num_cells_up + 1)] for j in range(2 * num_cells_right + 1)]
    self.set(0, 0, 1)

  def get(self, x, y):
    return self.pdf[self.num_cells_right + x][self.num_cells_up + y]

  def set(self, x, y, value):
    self.pdf[self.num_cells_right + x][self.num_cells_up + y] = value

  def get_location_from_indices(self, xidx, yidx):
    return [self.center_x + self.delta_x * (xidx - self.num_cells_right),
            self.center_y + self.delta_y * (yidx - self.num_cells_up)]

  def get_estimated_location(self):
    best_x = 0
    best_y = 0
    for i in range(2 * self.num_cells_right + 1):
      for j in range(2 * self.num_cells_up + 1):
        if self.pdf[i][j] > self.pdf[best_x][best_y]:
          best_x = i
          best_y = j
    return self.get_location_from_indices(best_x, best_y)

  def move(self, distance_moved, motion_uncertainty, heading, heading_uncertainty):
    """returns new_grid"""
    x_moved = distance_moved * math.sin(heading)
    y_moved = distance_moved * math.cos(heading)
    new_grid = Grid(self.center_x + x_moved, self.center_y + y_moved)
    for xnew_idx in range(2 * new_grid.num_cells_right + 1):
      for ynew_idx in range(2 * new_grid.num_cells_up + 1):
        probability = 0
        (xnew, ynew) = new_grid.get_location_from_indices(xnew_idx, ynew_idx)
        for xold_idx in range(2 * self.num_cells_right + 1):
          for yold_idx in range(2 * self.num_cells_up + 1):
            (xold, yold) = self.get_location_from_indices(xold_idx, yold_idx)
            dist = util.distance(xold, yold, xnew, ynew)
            dist_probability = util.probability_normal(distance_moved, motion_uncertainty, 
              dist)
            current_heading = util.heading(xold, yold, xnew, ynew)
            heading_diff = util.smallest_angle_difference(current_heading, heading)
            heading_probability = util.probability_normal(0, heading_uncertainty, heading_diff)
            probability += self.pdf[xold_idx][yold_idx] * dist_probability * heading_probability
        new_grid.pdf[xnew_idx][ynew_idx] = probability
        print xnew_idx, ynew_idx, new_grid.pdf[xnew_idx][ynew_idx]
    return new_grid

  @staticmethod
  def update_distances(grid0, grid1, distances, distance_uncertainties):
    """returns new_grid0, new_grid1"""
    pass


class GridLocalize(Localize_Interface):

  def __init__(self, start_positions, motion_uncertainties, angle_uncertainties, 
    distance_uncertainties):
    self.grids = []
    self.grids.append(Grid(start_positions[0][0], start_positions[0][1]))
    self.grids.append(Grid(start_positions[1][0], start_positions[1][1]))
    self.motion_uncertainties = motion_uncertainties
    self.angle_uncertainties = angle_uncertainties
    self.distance_uncertainties = distance_uncertainties

  def measure_movement(self, distances_moved, directions):
    for i in range(2):
      self.grids[i] = self.grids[i].move(distances_moved[i], 
        distances_moved[i] * self.motion_uncertainties[i], 
        directions[i], self.angle_uncertainties[i])

  def measure_distance(self, distances):
    dist_uncertainties = [self.distance_uncertainties[i] * distances[i] for i in range(2)]
    (self.grids[0], self.grids[1]) = Grid.update_distances(self.grids[0], self.grids[1], 
      distances, self.distance_uncertainties)

  def get_pose_estimate(self, rover_idx):
    return self.grids[rover_idx].get_estimated_location()
    
if __name__ == "__main__":
  g = GridLocalize([[0,1.34], [1,2.1]], [0.1, 0.1], [0.5, 0.5], [0.1, 0.1])
  print g.get_pose_estimate(0)
  print g.get_pose_estimate(1)
  g.measure_movement([1,1],[0,0])
  print g.get_pose_estimate(0)
  print g.get_pose_estimate(1)

