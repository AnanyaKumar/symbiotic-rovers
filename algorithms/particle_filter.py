
import math
import numpy as np
import scipy.stats
import random
import bisect

from localize_interface import Localize_Interface
import util

class ParticleLocalize(Localize_Interface):

  num_particles = 1000
  particles = []

  @util.overrides(Localize_Interface)
  def __init__(self, start_positions, motion_uncertainties, angle_uncertainties,
    distance_uncertainties, delta_heading_uncertainties):
    self.positions = start_positions
    self.motion_uncertainties = motion_uncertainties
    self.angle_uncertainties = angle_uncertainties
    self.distance_uncertainties = distance_uncertainties
    self.delta_heading_uncertainties = delta_heading_uncertainties
    particle = [start_positions[0][0], start_positions[0][1], start_positions[1][0], 
      start_positions[1][1]]
    self.particles = [particle for i in range(self.num_particles)]

  @util.overrides(Localize_Interface)
  def measure_movement(self, distances_moved, directions):
    def new_particle(old_particle):
      my_distance = np.random.normal(distances_moved[0], math.sqrt(self.motion_uncertainties[0]))
      my_angle = np.random.normal(directions[0], math.sqrt(self.angle_uncertainties[0]))
      [x_old, y_old] = [old_particle[0], old_particle[1]]
      [x0_new, y0_new] = [x_old + my_distance * math.cos(my_angle), y_old + my_distance * math.sin(my_angle)]
      my_distance = np.random.normal(distances_moved[1], math.sqrt(self.motion_uncertainties[1]))
      my_angle = np.random.normal(directions[1], math.sqrt(self.angle_uncertainties[1]))
      [x_old, y_old] = [old_particle[2], old_particle[3]]
      [x1_new, y1_new] = [x_old + my_distance * math.cos(my_angle), y_old + my_distance * math.sin(my_angle)]
      return [x0_new, y0_new, x1_new, y1_new]
    self.particles = [new_particle(p) for p in self.particles]

  @util.overrides(Localize_Interface)
  def measure_distance(self, distances, headings):
    def probability(particle):
      [x0, y0] = [particle[0], particle[1]]
      [x1, y1] = [particle[2], particle[3]]
      my_distance = math.sqrt((x1-x0)**2 + (y1-y0)**2)
      return scipy.stats.norm.pdf(my_distance, distances[0], math.sqrt(self.distance_uncertainties[0]))
    probabilities = [probability(p) for p in self.particles]

    prefix_sum = [0]
    for i in range(self.num_particles):
      prefix_sum.append(prefix_sum[i] + probabilities[i])
    tot_p = prefix_sum[len(probabilities)]

    new_particles = []
    r = random.uniform(0, tot_p / self.num_particles)
    index = 1
    for i in range(self.num_particles):
      while prefix_sum[index] < r:
        index += 1
      assert index <= self.num_particles
      new_particles.append(self.particles[index-1])
      r += tot_p / self.num_particles
    self.particles = new_particles

  @util.overrides(Localize_Interface)
  def get_pose_estimate(self, rover_idx):
    s = []
    for i in range(4):
      s.append(sum([p[i] for p in self.particles]) / self.num_particles)
    if rover_idx == 0:
      return [s[0], s[1]]
    else:
      return [s[2], s[3]]

  @util.overrides(Localize_Interface)
  def get_possible_pose_estimates(self, rover_idx):
    (best_x, best_y) = self.get_pose_estimate(rover_idx)
    return [[best_x], [best_y]]

if __name__ == "__main__":
  g = ParticleLocalize([[0,0], [0,0]], [0.1, 0.1], [0.04, 0.04], [0.0001, 0.0001], [0.1, 0.1])
  print g.get_pose_estimate(0)
  print g.get_pose_estimate(1)

  g.measure_movement([10, 10],[- math.pi / 4 - 0.1, math.pi / 4 + 0.1])
  print g.get_pose_estimate(0)
  print g.get_pose_estimate(1)

  g.measure_distance([9.33, 9.33], [0, 0])
  print g.get_pose_estimate(0)
  print g.get_pose_estimate(1)
