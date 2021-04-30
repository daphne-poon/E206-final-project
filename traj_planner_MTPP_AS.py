# E206 Motion Planning

# Simple planner
# C Clark
import copy
import math
import dubins
import random
import operator
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib import cm
from matplotlib.colors import ListedColormap

from traj_planner_utils import *
import numpy as np


class Node:
    LARGE_NUMBER = 9999999

    def __init__(self, state):
        '''
        state: array of state values, [time, x, y]
        parent_node: node being expanded
        desired_state: position to reach
        tree: index of tree that the node is associated with
        node_type: type of node, can be uninitialized, empty, obstacle, chaser, evader
        '''
        self.state = state
        self.parent_node = None
        self.g_cost = 0
        self.h_cost = 0
        self.f_cost = self.LARGE_NUMBER
        self.tree = None
        self.in_tree = False
        self.type = 'uninitialized'

    def manhattan_distance_to_node(self, node):
        return abs(self.state[1] - node.state[1]) + abs(self.state[2] - node.state[2])

    def manhattan_distance_to_state(self, state):
        return abs(self.state[1] - state[1]) + abs(self.state[2] - state[2])

    def euclidean_distance_to_state(self, state):
        return math.sqrt((self.state[1] - state[1]) ** 2 + (self.state[2] - state[2]) ** 2)

    def set_obstacle(self):
        self.type = 'obstacle'

    def set_empty(self):
        self.type = 'empty'

    def set_chaser(self):
        self.type = 'chaser'

    def set_evader(self):
        self.type = 'evader'

    def update_node(self, parent_node, chaser_state, tree):
        self.parent_node = parent_node
        if parent_node is not None:
            self.g_cost = parent_node.g_cost + 1  # parent_node.manhattan_distance_to_state(self.state)
        else:
            self.g_cost = 1
        self.h_cost = self.euclidean_distance_to_state(chaser_state)
        self.f_cost = self.g_cost + self.h_cost
        self.tree = tree
        self.in_tree = True

    def get_color(self):
        """ Gets the corresponding color code for the current node type.
            Returns: A int that corresponds to the color to plot in imshow().
        """
        if self.type == 'empty' or self.type == 'uninitialized':
            return 0
        elif self.type == 'chaser':
            return 1
        elif self.type == 'evader':
            return 2
        elif self.type == 'obstacle':
            return 3
        else:
            return 4


class MTPP:
    DIST_TO_GOAL_THRESHOLD = 0.5  # m
    CHILDREN_DELTAS = [-0.5, -0.25, 0.0, 0.25, 0.5]
    DISTANCE_DELTA = 1  # m
    EDGE_TIME = 2  # s
    LARGE_NUMBER = 9999999
    N = 10

    def __init__(self):
        self.grid = self.contruct_grid()
        self.cmap = self.create_cmap()

    def contruct_grid(self):
        '''
        grid: empty NxN array of uninitialized nodes
        '''
        grid = [[Node([0, i, j]) for i in range(self.N)] for j in range(self.N)]

        return grid

    def initial_path_finding(self, chaser_state, evader_state):
        """ Construct a trajectory in the X-Y space and in the time-X,Y,Theta space.
            Arguments:
            Returns:
              traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
        """
        print(chaser_state)
        self.chaser_state = chaser_state
        self.evader_state = evader_state
        self.tree_roots = []
        self.open_set = []
        self.leaf_set = []
        self.path_found = False

        # initial path finding
        self.tree_count = 0
        node_list = self.get_available_nodes(evader_state)

        for node in node_list:
            if node.type == 'chaser':  # TODO: fix edgecase
                node.update_node(None, chaser_state, self.tree_count)
                self.open_set.clear()
                return self.build_traj(node)

            node.set_empty()
            node.update_node(None, chaser_state, self.tree_count)
            self.tree_roots.append(node)
            self.open_set.append(node)
            self.tree_count += 1

        while True:
            if len(self.open_set) == 0:
                print("Error: Initial path not found")
            priority_node = self.get_highest_priority_node()
            self.tree_count = priority_node.tree
            node = self.expand_node(priority_node)
            if self.path_found:
                self.open_set.clear()
                return self.build_traj(node)

    def get_available_nodes(self, state_to_expand):
        '''
        returns list of nodes avalible for expansion
        '''
        node_list = []
        for i in [1, 2]:
            for delta in [self.DISTANCE_DELTA, -self.DISTANCE_DELTA]:
                state = copy.deepcopy(state_to_expand)
                state[i] = state[i] + delta
                # checks for wall
                if abs(state[i]) >= self.N:
                    break
                # check for obstacles
                node = self.grid[int(state[1])][int(state[2])]
                if node.type != 'obstacle':
                    node_list.append(node)
        return node_list

    def get_highest_priority_node(self):
        '''
        "pops" a node from the open set based on priority
        '''
        list_min = min(self.open_set, key=operator.attrgetter('f_cost'))  # TODO: check if we should use f_cost instead
        priority_node = copy.deepcopy(list_min)
        self.open_set.remove(list_min)
        return priority_node

    def expand_node(self, node_to_expand):
        '''
        expands node by adding appropriate nodes to the open and leaf set
        '''
        node_list = self.get_available_nodes(node_to_expand.state)
        for node in node_list:
            temp_cost = node_to_expand.g_cost + node_to_expand.manhattan_distance_to_state(
                node.state) + node.euclidean_distance_to_state(self.chaser_state)
            if node.type == 'chaser':
                print("found chaser at", node.state)
                node.update_node(node_to_expand, self.chaser_state, self.tree_count)
                self.path_found = True
                return node
            elif node.f_cost > temp_cost:
                if node.type != 'evader':
                    node.set_empty()
                node.update_node(node_to_expand, self.chaser_state, self.tree_count)
                self.open_set.append(node)
                if node in (self.leaf_set):
                    self.leaf_set.remove(node)
            if node.f_cost < temp_cost and node.tree != node_to_expand.tree:
                self.leaf_set.append(node)
                if node_to_expand not in self.leaf_set:
                    self.leaf_set.append(node_to_expand)

    def calculate_edge_distance(self, state, parent_node: Node):
        """
        Determine the traj, traj_distance for an edge,
        then check for collisions on traj with the collision_found function.
        If one is found return a LARGE_NUMBER.
        """
        traj, traj_distance = construct_dubins_traj(parent_node.state, state)
        if not collision_found(traj, self.objects, self.walls):
            return parent_node.manhattan_distance_to_state(state)

        return MTPP.LARGE_NUMBER

    def build_traj(self, goal_node):

        node_list = []
        node_to_add = goal_node
        while node_to_add != None:
            print(
                f'x: {node_to_add.state[1]}, y: {node_to_add.state[2]}, fcost: {node_to_add.f_cost}, gcost: {node_to_add.g_cost}, hcost: {node_to_add.h_cost}')
            node_list.insert(0, node_to_add)
            node_to_add = node_to_add.parent_node

        node_list.reverse()

        discretized_traj = []
        traj = []
        parent_time = None
        traj_point_0 = [0, 0, 0, 0]
        traj_point_1 = [0, 0, 0, 0]
        for i in range(1, len(node_list)):
            node_A = node_list[i - 1]
            node_B = node_list[i]
            traj_point_0[3] = traj_point_1[3]
            traj_point_0[0:3] = node_A.state
            traj_point_1[0:3] = node_B.state
            traj_point_1[3] = math.atan2(traj_point_1[2] - traj_point_0[2], traj_point_1[1] - traj_point_0[1])
            if len(traj) > 0:
                parent_time = traj[-1][0]
            edge_traj, edge_traj_distance = construct_dubins_traj(traj_point_0, traj_point_1, parent_time=parent_time)
            traj = traj + edge_traj
            discretized_traj = discretized_traj + [node_list[i].state]
        return traj, discretized_traj

        # TODO: add final step to evader

    def collision_found(self, node_1, node_2):
        """ Return true if there is a collision with the traj between 2 nodes and the workspace
            Arguments:
              node_1 (Node): A node with the first state of the traj - Time, X, Y, Theta (s, m, m, rad).
              node_2 (Node): A node with the second state of the traj - Time, X, Y, Theta (s, m, m, rad).
              objects (list of lists): A list of object states - X, Y, radius (m, m, m).
              walls (list of lists): A list of walls defined by end points - X0, Y0, X1, Y1, length (m, m, m, m, m).
            Returns:
              collision_found (boolean): True if there is a collision.
        """
        traj, traj_distance = construct_dubins_traj(node_1.state, node_2.state)
        return collision_found(traj, self.objects, self.walls)

    def show_current_grid(self, traj=None):
        """ Plots the current state of the grid.
            Arguments:
              traj (list of lists): If not None, will plot the traj on top of the grid.
        """

        color_array = []

        for level in self.grid:
            color_array_level = [node.get_color() for node in level]
            color_array.append(color_array_level)

        print(self.grid[1][1].type)
        if traj is not None:
            for point in traj:
                color_array[point[1]][point[2]] = 4

        print(color_array)
        plt.imshow(color_array, cmap=self.cmap, origin='lower', interpolation='none', alpha=1,
                   extent=(0, self.N, 0, self.N), vmin=0, vmax=5)
        major_ticks = np.arange(0, self.N + 1, 1)

        # Make legend
        chaser_p = mpatches.Patch(color='red', label='Chaser')
        evader_p = mpatches.Patch(color='blue', label='Evader')
        obstacle_p = mpatches.Patch(color='black', label='Obstacle')
        path_p = mpatches.Patch(color='yellow', label='Path')
        plt.legend(handles=[chaser_p, evader_p, obstacle_p, path_p], loc='upper left')
        plt.xticks(major_ticks)
        plt.yticks(major_ticks)
        plt.grid(True, which='both')
        plt.show()

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
        yellow = np.array([256 / 256, 256 / 256, 0 / 256, 1])
        newcolors[200:, :] = yellow
        newcmp = ListedColormap(newcolors)

        return newcmp


if __name__ == '__main__':
    # for i in range(0, 5):
    planner = MTPP()
    chaser_state = [0, 1, 1]
    print(chaser_state[1], chaser_state[2])
    planner.grid[chaser_state[1]][chaser_state[2]].set_chaser()
    evader_state = [0, 8, 8]
    planner.grid[evader_state[1]][evader_state[2]].set_evader()
    # tp1 = [300, random.uniform(-maxR + 1, maxR - 1), random.uniform(-maxR + 1, maxR - 1), 0]

    # initial path finding
    initial_path, discretized_path = planner.initial_path_finding(chaser_state, evader_state)

    maxR = 10
    walls = [[-maxR, maxR, maxR, maxR, 2 * maxR], [maxR, maxR, maxR, -maxR, 2 * maxR],
             [maxR, -maxR, -maxR, -maxR, 2 * maxR], [-maxR, -maxR, -maxR, maxR, 2 * maxR]]

    objects = []
    # if len(initial_path) > 0:
    # plot_traj(initial_path, initial_path, objects, walls)
    # print(discretized_path)
    planner.show_current_grid(discretized_path)
    # begin moving robot

    # objects = []
    # num_objects = 25
    # #TODO: add obstacles

    # for j in range(0, num_objects):
    #     obj = [random.uniform(-maxR + 1, maxR - 1), random.uniform(-maxR + 1, maxR - 1), 0.5]
    #     while (abs(obj[0] - tp0[1]) < 1 and abs(obj[1] - tp0[2]) < 1) or (
    #             abs(obj[0] - tp1[1]) < 1 and abs(obj[1] - tp1[2]) < 1):
    #         obj = [random.uniform(-maxR + 1, maxR - 1), random.uniform(-maxR + 1, maxR - 1), 0.5]
    #     objects.append(obj)

    # if len(initial_path) > 0:
    #     plot_initial_path(traj, traj, objects, walls)
    #     #plot_traj(traj, traj, objects, walls)
