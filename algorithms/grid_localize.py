
import math

from localize_interface import Localize_Interface
import util
import grid

class GridLocalize(Localize_Interface):

  @util.overrides(Localize_Interface)
  def __init__(self, start_positions, motion_uncertainties, angle_uncertainties,
    distance_uncertainties):
    self.grids = []    
    for i in range(2):
      self.grids.append(grid.Grid())
      self.grids[i].initialize(start_positions[i][0], start_positions[i][1], 0.1, 0.1, 10, 10)
    self.motion_uncertainties = motion_uncertainties
    self.angle_uncertainties = angle_uncertainties
    self.distance_uncertainties = distance_uncertainties

  @util.overrides(Localize_Interface)
  def measure_movement(self, distances_moved, directions):
    for i in range(2):
      self.grids[i].move(distances_moved[i], distances_moved[i] * self.motion_uncertainties[i],
        directions[i], self.angle_uncertainties[i])
      self.grids[i].swap();

  @util.overrides(Localize_Interface)
  def measure_distance(self, distances):
    dist_uncertainties = [self.distance_uncertainties[i] * distances[i] for i in range(2)]
    eps = 0.05
    if self.distance_uncertainties[0] < self.distance_uncertainties[1] - eps:
      distance = distances[0]
      distance_uncertainty = dist_uncertainties[0]
    elif self.distance_uncertainties[1] < self.distance_uncertainties[0] - eps:
      distance = distances[1]
      distance_uncertainty = dist_uncertainties[1]
    else:
      distance = (distances[0] + distances[1]) / 2
      distance_uncertainty = (dist_uncertainties[0] + dist_uncertainties[1]) / 2

    for i in range(2):
      self.grids[i].update_distance(self.grids[(i+1)%2], distance, distance_uncertainty)
    for i in range(2):
      self.grids[i].swap()

  @util.overrides(Localize_Interface)
  def get_pose_estimate(self, rover_idx):
    p = self.grids[rover_idx].get_estimated_location()
    return (p.x, p.y)

if __name__ == "__main__":
  g = GridLocalize([[0,0], [0,0]], [0.1, 0.1], [0.5, 0.5], [0.1, 0.1])
  print g.get_pose_estimate(0)
  print g.get_pose_estimate(1)
  g.measure_movement([2,1],[0,0])
  print g.get_pose_estimate(0)
  print g.get_pose_estimate(1)
  g.measure_distance([0.5, 0.5])
  print g.get_pose_estimate(0)
  print g.get_pose_estimate(1)
