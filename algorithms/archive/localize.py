class Localize:

  # Only supports 2 rovers right now
  def __init__(self, num_rovers, start_positions, motion_uncertainty, distance_uncertainty):
    self.num_rovers = num_rovers
    self.postions = start_positions
    self.motion_uncertainty = motion_uncertainty
    self.distance_uncertainty = distance_uncertainty

  # Direction is in degrees taken clockwise from north
  def move(self, rover_idx, distance, direction):
    print self.num_rovers

  def measure_distance(distance):
    pass

  def get_pose_estimate(rover_idx):
    pass

  def get_pose_estimates():
    pass
