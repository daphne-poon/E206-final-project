# E206 Motion Planning
import matplotlib.patches as mpatches
import numpy as np
from matplotlib import cm
from matplotlib.colors import ListedColormap

from node import Node
from traj_planner_utils import *

class Map:

    def __init__(self, N, num_evaders, chaser_state, evader_state):
        self.N = N
        self.contruct_grid()
        self.chaser_state = chaser_state
        self.evader_state = evader_state
        self.num_evaders = num_evaders
        self.cmap = self.create_cmap()

    def contruct_grid(self):
        """ Construct a trajectory in the X-Y space and in the time-X,Y,Theta space.
            Returns:
              grid (list of lists): An empty NxN array of uninitialized nodes.
        """
        self.grid = [[Node([0, i, j]) for j in range(self.N)] for i in range(self.N)]

    def get_evader_node(self):
        return self.grid[self.evader_state[1]][self.evader_state[2]]

    def get_chaser_node(self):
        return self.grid[self.chaser_state[1]][self.chaser_state[2]]

    ## FOR PRINTING PURPOSES

    def create_cmap(self):
        """ Plots the current state of the grid.
            Returns:
              newcmp: A custom colormap.
        """
        viridis = cm.get_cmap('viridis', 256)
        newcolors = viridis(np.linspace(0, 1, 256))
        white = np.array([256 / 256, 256 / 256, 256 / 256, 1])
        newcolors[:25, :] = white
        red = np.array([256 / 256, 0 / 256, 0 / 256, 1])
        newcolors[30:, :] = red
        blue = np.array([0 / 256, 0 / 256, 256 / 256, 1])
        newcolors[100:, :] = blue
        black = np.array([0 / 256, 0 / 256, 0 / 256, 1])
        newcolors[150:, :] = black
        yellow = np.array([256 / 256, 256 / 256, 0 / 256, 1])
        newcolors[200:, :] = yellow
        newcmp = ListedColormap(newcolors)

        return newcmp

    def show_current_grid(self, traj=None, tree=False):
        """ Plots the current state of the grid.
            Arguments:
              traj (list of lists): If not None, will plot the traj on top of the grid.
        """
        color_array = []
        if tree:
            for level in self.grid:
                color_array_level = [node.get_tree() for node in level]
                color_array.append(color_array_level)

            viridis = cm.get_cmap('viridis', 256)
            newcolors = viridis(np.linspace(0, 1, 256))
            white = np.array([256 / 256, 256 / 256, 256 / 256, 1])
            newcolors[:25, :] = white
            newcmp = ListedColormap(newcolors)

            plt.imshow(color_array, cmap=newcmp, origin='lower', interpolation='none', alpha=1,
                       extent=(0, self.N, 0, self.N))
        else:
            for level in self.grid:
                color_array_level = [node.get_color() for node in level]
                color_array.append(color_array_level)

            if traj is not None:
                for point in traj:
                    color_array[point[1]][point[2]] = 4

            # color_array_t = np.array(color_array).T.tolist()
            plt.imshow(color_array, cmap=self.cmap, origin='lower', interpolation='none', alpha=1,
                       extent=(0, self.N, 0, self.N), vmin=0, vmax=5)
            chaser_p = mpatches.Patch(color='red', label='Chaser')
            evader_p = mpatches.Patch(color='blue', label='Evader')
            obstacle_p = mpatches.Patch(color='black', label='Obstacle')
            path_p = mpatches.Patch(color='yellow', label='Path')
            plt.legend(handles=[chaser_p, evader_p, obstacle_p, path_p], loc='upper left')

        # Make legend
        major_ticks = np.arange(0, self.N + 1, 1)

        plt.xticks(major_ticks)
        plt.yticks(major_ticks)
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid(True, color='black', which='both')
        plt.show()
