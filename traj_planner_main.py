# E206 Motion Planning

# Combined trajectory planner (evader and chaser)
# Daphne Poon and Sabrina Shen
import random
import time
from traj_planner_utils import *

import numpy as np
from matplotlib import cm
import matplotlib.patches as mpatches
from matplotlib.colors import ListedColormap

from traj_planner_APF import APF_Planner
from traj_planner_MTPP import MTPP


class Environment():
    DIST_TO_GOAL_THRESHOLD = 0.5  # m
    N = 20
    NUM_EVADERS = 1 # ez

    def __init__(self):
        self.evader_planners = []
        self.chaser_planner = MTPP(self.N, self.DIST_TO_GOAL_THRESHOLD)
        self.grid = []
        pass

    def initialize_evaders(self, num):
        for _ in range(num):
            self.evader_planners.append(APF_Planner())
        pass

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
            plt.imshow(color_array, cmap=cmap, origin='lower', interpolation='none', alpha=1,
                       extent=(0, N, 0, N), vmin=0, vmax=5)
            chaser_p = mpatches.Patch(color='red', label='Chaser')
            evader_p = mpatches.Patch(color='blue', label='Evader')
            obstacle_p = mpatches.Patch(color='black', label='Obstacle')
            path_p = mpatches.Patch(color='yellow', label='Path')
            plt.legend(handles=[chaser_p, evader_p, obstacle_p, path_p], loc='upper left')

        # Make legend
        major_ticks = np.arange(0, N + 1, 1)

        plt.xticks(major_ticks)
        plt.yticks(major_ticks)
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid(True, color='black', which='both')
        plt.show()

    def build_dynamic_path_to_target(self):
        pass

    def target_is_reached(self):
        evader_node = self.get_evader_node()
        if evader_node.euclidean_distance_to_state(self.chaser_state) <= 1.5:
            return True
        return False

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

    def build_dynamic_path_to_target(self, chaser_state, evader_state):
        _, discretized_path = self.initial_path_finding(chaser_state, evader_state)
        self.show_current_grid(discretized_path)
        self.show_current_grid(tree=True)

        while not self.target_is_reached():
            self.chaser_state = self.update_chaser_position()
            self.evader_state = self.update_evader_position()
            _, discretized_path = self.correct_path()
            self.show_current_grid(discretized_path)
            self.show_current_grid(tree=True)

        print("TARGET REACHED!!!!")
        self.show_current_grid(discretized_path)


if __name__ == '__main__':
    environment = Environment()

    # initialize chaser and evader
    chaser_state = [0, 4, 2]
    environment.chaser_planner.grid[chaser_state[1]][chaser_state[2]].set_chaser()
    evader_state = [0, 15, 13]
    environment.chaser_planner.grid[evader_state[1]][evader_state[2]].set_evader()

    # initialize obstacles
    random.seed(time.time())
    for _ in range(2 * environment.chaser_planner.N):
        rand_x = random.randint(0, environment.chaser_planner.N - 1)
        rand_y = random.randint(0, environment.chaser_planner.N - 1)
        if environment.chaser_planner.grid[rand_x][rand_y].type == 'uninitialized':
            environment.chaser_planner.grid[rand_x][rand_y].set_obstacle()

    environment.show_current_grid()

    environment.build_dynamic_path_to_target()
