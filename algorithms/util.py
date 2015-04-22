import math
from numpy.random import normal

def distance(x0, y0, x1, y1):
  return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

def heading(x0, y0, x1, y1):
  x_disp = x1 - x0
  y_disp = y1 - y0
  return math.atan2(x_disp, y_disp) # our coordinate system isn't the mathematical system

def probability_normal(mean, variance, value):
  sigma = math.sqrt(variance)
  u = (value - mean) / abs(sigma)
  y = (1 / (math.sqrt(2 * math.pi) * abs(sigma))) * math.exp(-u * u / 2)
  return y

def smallest_angle_difference(angle1, angle2):
  angle_diff = angle2 - angle1
  if (angle_diff > math.pi):
    angle_diff -= math.pi
  elif (angle_diff < -math.pi):
    angle_diff += math.pi
  return angle_diff
