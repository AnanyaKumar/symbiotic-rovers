import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

# Utilities

def degree_to_rad(degree):
  degree = degree / 180.0 * math.pi
  return degree

def component_add(*args):
  def helper_add(array1, array2):
    return [x + y for (x,y) in zip(array1, array2)]
  return reduce(args, helper_add)
 
def average_array(*args):
  return [float(x) / len(args) for x in component_add(*args)]

# Functions to calculate position

def next_position_odometry(current_x, current_y, distance_walked, direction_walked):
  """Returns the next position given odometry information"""
  x_disp = distance_walked * math.sin(degree_to_rad(direction_walked))
  y_disp = distance_walked * math.cos(degree_to_rad(direction_walked))
  return (current_x + x_disp, current_y + y_disp)


def positions_from_odometry(distances_walked, directions_walked, start_x = 0, start_y = 0):
  """Returns positions given the distance and direction walked at each time step.

  Args:
    distances_walked: array representing distance walked each time step.
    directions_walked: array representing direction walked each time step.
      Direction is angle (degree) from North (positive Y), clockwise.
    start_x: initial x-coordinate.
    start_y: initial y-coordinate.

  Returns:
    (xarray, yarray) representing positions, includes start and end positions.
    xarray: array of x-coordinates of positions.
    yarray: array of y-coordinates of positions.
  """
  xarray = [start_x]
  yarray = [start_y]
  for i in range(len(distances_walked)):
    (newx, newy) = next_position_odometry(xarray[i], yarray[i], distances_walked[i], 
      directions_walked[i])
    xarray.append(newx)
    yarray.append(newy)
  return (xarray, yarray)

def positions_from_headings(distances, headings):
  """Returns positions of object given the distance and heading to an object at each time step.

  Args:
    distances: array representing distances to object at each time step.
    headings: array representing heading to object at each time step.
      Heading is angle (in degrees) from North (positive Y), measured clockwise.

  Returns:
    (xarray, yarray) representing x-coordinates and y-coordinates of object's positions.
  """
  xarray = [distances[i] * math.sin(degree_to_rad(headings[i]-180)) for i in range(len(distances))]
  yarray = [distances[i] * math.cos(degree_to_rad(headings[i]-180)) for i in range(len(distances))]
  return (xarray, yarray)

def positions_triangulate():
  pass

# Plotting functions

def plot_points(xarray, yarray):
  # Make array of colors
  colors = cm.Greys(np.linspace(0, 1, len(xarray)))
  # Plot graph
  for i in range(len(xarray)):
    plt.scatter(xarray[i], yarray[i], c=colors[i], s = 100)
  plt.show()

if __name__ == "__main__":
  # (xarray, yarray) = positions_from_odometry(
  #   [21.089494235, 19.87638173, 21.27612693, 18.1966875, 17.076891345, 16.79694231, 16.890258655, 21.369443275000002, 21.27612693, 19.22316731],
  #   [324.6666666666667, 327.0, 324.3333333333333, 270.0, 99.33333333333333, 105.0, 96.0, 196.0, 202.0, 205.0])
  (xarray, yarray) = positions_from_headings([0,15,30,45,60,55,50,45,30,15,0],
    [0,165,148,151,153,164,169,226,191,193,0])
  plot_points(xarray, yarray)
