# E206 Motion Planning
import matplotlib.patches as mpatches
import numpy as np
from matplotlib import cm
from matplotlib.colors import ListedColormap

from node import Node
from traj_planner_utils import *


class Map:

    def __init__(self, N, num_evaders, chaser_state, evader_states):
        self.N = N
        self.contruct_grid()
        self.chaser_state = chaser_state
        self.evader_states = evader_states
        self.num_evaders = num_evaders
        self.obstacles = []
        self.cmap = self.create_cmap()
        self.current_evader = 0  # the current evader we are trying to "catch"
        self.dead_evaders = []  # list of evaders that have already been caught

    def contruct_grid(self):
        """ Construct a trajectory in the X-Y space and in the time-X,Y,Theta space.
            Returns:
              grid (list of lists): An empty NxN array of uninitialized nodes.
        """
        self.grid = [[Node([0, i, j]) for j in range(self.N)] for i in range(self.N)]

    def get_evader_node(self, id=0):
        return self.grid[self.evader_states[id][1]][self.evader_states[id][2]]

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

            # for level in self.grid:
            #     for node in level:
            #         if node.type == 'evader':
            #             color_array[node.state[1]][node.state[2]] = node.get_color()

            # color_array_t = np.array(color_array).T.tolist()
            plt.imshow(color_array, cmap=self.cmap, origin='lower', interpolation='none', alpha=1,
                       extent=(0, self.N, 0, self.N), vmin=0, vmax=5)
            chaser_p = mpatches.Patch(color='red', label='Chaser')
            evader_p = mpatches.Patch(color='blue', label='Evader')
            obstacle_p = mpatches.Patch(color='black', label='Obstacle')
            path_p = mpatches.Patch(color='yellow', label='Path')
            plt.legend(handles=[chaser_p, evader_p, obstacle_p, path_p], loc=(1.02, 0.5))

        # Make legend
        major_ticks = np.arange(0, self.N + 1, 1)

        plt.xticks(major_ticks)
        plt.yticks(major_ticks)
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid(True, color='black', which='both')
        plt.show()

    def add_obstacles(self, obstacles):
        self.obstacles = obstacles
        for obstacle in self.obstacles:
            self.grid[obstacle[0]][obstacle[1]].set_obstacle()
        return
