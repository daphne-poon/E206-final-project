# E206 Motion Planning

# Combined trajectory planner (evader and chaser)
# Daphne Poon and Sabrina Shen
import random

from map import Map
from node import Node
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

    def __init__(self, chaser_state, evader_state):
        self.map = Map(self.N, self.NUM_EVADERS, chaser_state, evader_state)
        self.evader_planners = []
        self.chaser_planner = MTPP(self.map)
        pass

    def initialize_evaders(self, num):
        for _ in range(num):
            self.evader_planners.append(APF_Planner())
        pass

    def target_is_reached(self):
        evader_node = self.map.get_evader_node()
        if evader_node.euclidean_distance_to_state(self.map.chaser_state) <= 1.5:
            return True
        return False

    def build_dynamic_path_to_target(self):
        self.sync_maps()
        _, discretized_path = self.chaser_planner.initial_path_finding()
        self.sync_maps(from_chaser=True)
        self.map.show_current_grid(discretized_path)
        self.map.show_current_grid(tree=True)

        while not self.target_is_reached():
            self.sync_maps()
            self.map.chaser_state = self.chaser_planner.update_chaser_position()
            self.map.evader_state = self.chaser_planner.update_evader_position()
            self.sync_maps(from_chaser=True)
            _, discretized_path = self.chaser_planner.correct_path()
            self.sync_maps(from_chaser=True)
            self.map.show_current_grid(discretized_path)
            self.map.show_current_grid(tree=True)

        print("TARGET REACHED!!!!")
        self.map.show_current_grid(discretized_path)

    def sync_maps(self, from_chaser=False, from_evader=False):
        if from_chaser:
            self.map = self.chaser_planner.map
            # self.evader_planners.map = self.chaser_planner.map
        elif from_evader:
            pass
            # self.map = self.evader_planner.map
            # self.chaser_planner.map = self.evader_planner.map
        else:
            self.chaser_planner.map = self.map
            # self.evader_planner.map = self.map


if __name__ == '__main__':
    chaser_state = [0, 4, 2]
    evader_state = [0, 15, 13]
    environment = Environment(chaser_state, evader_state)

    # initialize chaser and evader
    environment.map.grid[chaser_state[1]][chaser_state[2]].set_chaser()
    environment.map.grid[evader_state[1]][evader_state[2]].set_evader()

    # initialize obstacles
    random.seed(time.time())
    for _ in range(2 * environment.N):
        rand_x = random.randint(0, environment.N - 1)
        rand_y = random.randint(0, environment.N - 1)
        if environment.map.grid[rand_x][rand_y].type == 'uninitialized':
            environment.map.grid[rand_x][rand_y].set_obstacle()

    environment.build_dynamic_path_to_target()
