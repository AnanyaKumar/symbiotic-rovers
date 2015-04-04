import numpy.random
import math
import scipy

GRID_SAMPLE_WIDTH = 100
GRID_WIDTH = 1.0  # meters

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

  def init_particle_filter(x0, y0, x1, y1):
    # Create grid around (x0, y0), (x1, y1) of
    # (GRID_SAMPLE_WIDTH + 1) x (GRID_SAMPLE_WIDTH + 1) points. This represents
    # an equal division of the (GRID_WIDTH) x (GRID_WIDTH) square area around
    # (x0,y0) and (x1, y1)
    pass

  def dist(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)

  def ndens(mean, var, x):
    return scipy.stats.norm(mean, var).pdf(x)

  def sample_norm(mean, var, numpoints=1):
    return numpy.random.norm(mean, var, numpoints)

  def particle_filter(x_tm1, u_t, z_t):
    # u_t is [v1, theta1, v2, theta2]
    # z_t is [d, x0,y0, x1,y1]
    r1_x = x_tm1[0]
    r1_y = x_tm1[1]
    r2_x = x_tm1[2]
    r2_y = x_tm1[3]

    # calculate predicted position using controls
    pred_r1_x = r1_x + u_t[0] * math.cos(u_t[1])
    pred_r1_y = r1_y + u_t[0] * math.sin(u_t[1])

    pred_r2_x = r2_x + u_t[2] * math.cos(u_t[3])
    pred_r2_y = r2_y + u_t[2] * math.sin(u_t[3])

    m = 0
    opt_r1_x = 0
    opt_r1_y = 0
    opt_r2_x = 0
    opt_r2_y = 0

    for i1 in xrange(0, GRID_SAMPLE_WIDTH + 1):
      for j1 in xrange(0, GRID_SAMPLE_WIDTH + 1):
        for i2 in xrange(0, GRID_SAMPLE_WIDTH + 1):
          for j2 in xrange(0, GRID_SAMPLE_WIDTH + 1):
            w = (GRID_SAMPLE_WIDTH / 2.0)
            x1 = (i1 - w) / w * GRID_WIDTH
            y1 = (j1 - w) / w * GRID_WIDTH

            x2 = (i2 - w) / w * GRID_WIDTH
            y2 = (j2 - w) / w * GRID_WIDTH

            # estimate 3sigma as width
            p1 = ndens(0, GRID_WIDTH / 3, x1) * ndens(0, GRID_WIDTH / 3, y1)
            p2 = ndens(0, GRID_WIDTH / 3, x2) * ndens(0, GRID_WIDTH / 3, y2)

            pred_dist = dist(r1_x + x1, r1_y + y1, r2_x + x2, r2_y + y2)
            measured_dist = z_t[0]

            # give 1 as sigma for now
            pd = ndens(measured_dist, 1, pred_dist)

            if p1 * p2 * pd > m:
              m = p1 * p2 * pd
              opt_r1_x = x1
              opt_r1_y = y1
              opt_r2_x = x2
              opt_r2_y = y2

    return [opt_r1_x, opt_r1_y, opt_r2_x, opt_r2_y]
