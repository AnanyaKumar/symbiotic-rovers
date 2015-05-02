import math
from numpy.random import normal
from matplotlib import mpl, pyplot

def overrides(interface_class):
    def overrider(method):
        assert(method.__name__ in dir(interface_class))
        return method
    return overrider

def plot_grid(grid, image_name):
  fig = pyplot.figure(2)
  cmap2 = mpl.colors.LinearSegmentedColormap.from_list('my_colormap', ['white','black'], 256)
  img2 = pyplot.imshow(grid, interpolation='nearest', cmap = cmap2,origin='lower')
  fig.savefig(image_name)
