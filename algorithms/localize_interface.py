# Interface for localization algorithms

class Localize_Interface:

  def __init__(self, start_positions, motion_uncertainties, angle_uncertainties, 
    distance_uncertainties):
    """Initializes rover states with positions and uncertainty data.

    Args:
      start_positions: [ [x_0, y_0], [x_1, y_1] ] representing positions of the rovers, all floats
      motion_uncertainties: [o_0, o_1] uncertainties in odometry data for rovers. Represents 1
        variance of odometry measurement if rovers move 1 meter. All floats.
      angle_uncertainties: [a_0, a_1] uncertainties in angle data for rovers. Represents 1
        variance of angle measurement, in radians. All floats.
      distance_uncertainties: [m_1, m_2] uncertainties in distance measurement. Represents 1
        variance of distance measurement if rovers are 1 meter apart. All floats.
    """
    pass

  def measure_movement(self, distances_moved, directions):
    """Update estimates of the rovers' poses using rover odometry/angle data.

    Args:
      distances_moved: list containing distance moved (measured) by the rovers in meters. All 
        floats.
      directions: list containing heading (measured) of each rover. All floats.
    """
    pass

  def measure_distance(self, distances):
    """Update estimates of rovers' poses using measured distance between the rovers.
    
    Args:
      distances: [d_0, d_1] 
        d_0 is distance measured by rover 0 to rover 1, in meters (float).
        d_1 is distance measured by rover 1 to rover 0, in meters (float).
    """
    pass

  def get_pose_estimate(self, rover_idx):
    """Returns the best estimate of the specified rover's pose.

    Args:
      rover_idx: rover index (int).

    Returns:
      [x, y] representing the best estimate of the rover's pose. All floats.
    """
    pass
