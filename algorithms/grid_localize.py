
import math

from localize_interface import Localize_Interface
import util 
import grid_pdf

class GridLocalize(Localize_Interface):

  @util.overrides(Localize_Interface)
  def __init__(self, start_positions, motion_uncertainties, angle_uncertainties,
    distance_uncertainties, delta_heading_uncertainties):
    self.grids = []    
    for i in range(2):
      self.grids.append(grid_pdf.GridPdf())
      self.grids[i].initialize(start_positions[i][0], start_positions[i][1], 0.25, 0.25, 10, 10)
    self.motion_uncertainties = motion_uncertainties
    self.angle_uncertainties = angle_uncertainties
    self.distance_uncertainties = distance_uncertainties
    self.delta_heading_uncertainties = delta_heading_uncertainties

  @util.overrides(Localize_Interface)
  def measure_movement(self, distances_moved, directions):
    for i in range(2):
      self.grids[i].move(distances_moved[i], distances_moved[i] * self.motion_uncertainties[i],
        directions[i], self.angle_uncertainties[i])

  @util.overrides(Localize_Interface)
  def measure_distance(self, distances, headings):
    dist_uncertainties = [self.distance_uncertainties[i] * distances[i] for i in range(2)]
    eps = 0.8
    if self.distance_uncertainties[0] < self.distance_uncertainties[1] * eps:
      distance = distances[0]
      distance_uncertainty = dist_uncertainties[0]
    elif self.distance_uncertainties[1] < self.distance_uncertainties[0] * eps:
      distance = distances[1]
      distance_uncertainty = dist_uncertainties[1]
    else:
      distance = (distances[0] + distances[1]) / 2
      distance_uncertainty = (dist_uncertainties[0] + dist_uncertainties[1]) / 2
    grid_pdf.GridPdf.update_distance(self.grids[0], self.grids[1], distance, distance_uncertainty,
      headings[0], self.delta_heading_uncertainties[0])

  @util.overrides(Localize_Interface)
  def get_pose_estimate(self, rover_idx):
    p = self.grids[rover_idx].get_estimated_location()
    return (p.x, p.y)

  @util.overrides(Localize_Interface)
  def get_possible_pose_estimates(self, rover_idx):
    (best_x, best_y) = self.get_pose_estimate(rover_idx)
    return [[best_x], [best_y]]

  @staticmethod
  def make_2d_list(grid):
    x_dim = grid.get_dim_x()
    y_dim = grid.get_dim_y()
    return [[grid.get_probability(x,y) for y in range(y_dim)] for x in range(x_dim)]

  def plot_grids(self, image_number):
    grid0_list = GridLocalize.make_2d_list(self.grids[0])
    util.plot_grid(grid0_list, 'gridplots/' + '0-' + str(image_number) + '.png')
    grid1_list = GridLocalize.make_2d_list(self.grids[1])
    util.plot_grid(grid1_list, 'gridplots/' + '1-' + str(image_number) + '.png')

if __name__ == "__main__":
  g = GridLocalize([[0,0], [0,0]], [0.1, 0.1], [0.5, 0.5], [0.1, 0.1], [0.1, 0.1])
  g.plot_grids(1)
  print g.get_pose_estimate(0)
  print g.get_pose_estimate(1)

  g.measure_movement([2,1],[0,0])
  g.plot_grids(2)
  print g.get_pose_estimate(0)
  print g.get_pose_estimate(1)

  g.measure_movement([2,1],[0,0])
  g.plot_grids(3)
  print g.get_pose_estimate(0)
  print g.get_pose_estimate(1)

  g.measure_movement([2,1],[0,0])
  g.plot_grids(4)
  print g.get_pose_estimate(0)
  print g.get_pose_estimate(1)

  g.measure_movement([2,1],[0,0])
  g.plot_grids(5)
  print g.get_pose_estimate(0)
  print g.get_pose_estimate(1)

  g.measure_distance([0.5, 0.5], [0, 0])
  g.plot_grids(6)
  print g.get_pose_estimate(0)
  print g.get_pose_estimate(1)
