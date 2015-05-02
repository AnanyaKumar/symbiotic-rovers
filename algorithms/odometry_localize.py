
import math

from localize_interface import Localize_Interface
import util

class OdometryLocalize(Localize_Interface):

  @util.overrides(Localize_Interface)
  def __init__(self, start_positions, motion_uncertainties, angle_uncertainties,
    distance_uncertainties, delta_heading_uncertainties):
    self.positions = start_positions

  @util.overrides(Localize_Interface)
  def measure_movement(self, distances_moved, directions):
    def move(old_pose, distance_moved, direction):
      [x_old, y_old] = old_pose
      x_disp = distance_moved * math.cos(direction)
      y_disp = distance_moved * math.sin(direction)
      return [x_old + x_disp, y_old + y_disp]
    for i in range(len(self.positions)):
      self.positions[i] = move(self.positions[i], distances_moved[i], directions[i])

  @util.overrides(Localize_Interface)
  def measure_distance(self, distances, headings):
    pass

  @util.overrides(Localize_Interface)
  def get_pose_estimate(self, rover_idx):
    return self.positions[rover_idx]
