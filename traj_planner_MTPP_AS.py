# E206 Motion Planning

# Simple planner
# C Clark
import copy
import math
import dubins
import random
import matplotlib.pyplot as plt
from traj_planner_utils import *
import numpy as np


class Node:

    def __init__(self, state, parent_node, desired_state, tree):
        self.state = state
        self.parent_node = parent_node
        self.g_cost = parent_node.g_cost + self.manhattan_distance_to_node(parent_node)
        self.h_cost = self.manhattan_distance_to_state(desired_state)
        self.f_cost = self.g_cost + self.h_cost  # cost
        self.tree = tree

    def manhattan_distance_to_node(self, node):
        return abs(self.state[1] - node.state[1]) + abs(self.state[2] - node.state[2])

    def manhattan_distance_to_state(self, state):
        return abs(self.state[1] - state[1]) + abs(self.state[2] - state[2])

    def euclidean_distance_to_state(self, state):
        return math.sqrt((self.state[1] - state[1]) ** 2 + (self.state[2] - state[2]) ** 2)


class MTPP:
    DIST_TO_GOAL_THRESHOLD = 0.5  # m
    CHILDREN_DELTAS = [-0.5, -0.25, 0.0, 0.25, 0.5]
    DISTANCE_DELTA = 1.5  # m
    EDGE_TIME = 2  # s
    LARGE_NUMBER = 9999999

    def __init__(self):
        self.fringe = []

    def construct_traj(self, initial_state, desired_state, objects, walls, grid_size):
        """ Construct a trajectory in the X-Y space and in the time-X,Y,Theta space.
            Arguments:
              traj_point_0 (list of floats): The trajectory's first trajectory point with time, X, Y, Theta (s, m, m, rad).
              traj_point_1 (list of floats): The trajectory's last trajectory point with time, X, Y, Theta (s, m, m, rad).
            Returns:
              traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
        """
        self.fringe = []  # A LIST OF NODES
        self.desired_state = desired_state
        self.objects = objects
        self.walls = walls
        self.goal_found = False
        self.tree_roots = []
        self.open_set = []
        self.grid = np.zeros((grid_size, grid_size))

        # initial path finding
        # TODO: refactor into method, and add grid funtionality
        initial_node = self.create_initial_node(initial_state)
        children_list = self.get_children(initial_node)
        tree_count = 0
        for child in children_list:
            traj, traj_distance = construct_dubins_traj(initial_node.state, child.state)
            if not collision_found(traj, self.objects, self.walls):
                child.tree = tree_count
                tree_count += 1
                self.tree_roots.append(child.tree)
                self.open_set.append(child)

        # while loop until hunter is in open set
        while not self.hunter_in_open_set():
            # for node in self.open_set:
            node_to_expand = self.get_highest_priority_node()
            self.expand_nodes(node_to_expand)

        # loop to find goal
        while True:
            # get the top node, then expand it
            current_node = self.get_best_node_on_fringe()
            print(
                f'x: {current_node.state[1]}, y: {current_node.state[2]}, desired state: {desired_state}, fcost: {current_node.f_cost}, hcost: {current_node.h_cost}')
            # check to see if it connects to the goal
            traj, traj_distance = construct_dubins_traj(current_node.state, self.desired_state)
            if not collision_found(traj, self.objects, self.walls):
                # if it connects, yay! build and return the traj.
                goal_node = self.generate_goal_node(current_node, self.desired_state)
                return self.build_traj(goal_node)
            else:
                # if it doesn't connect, add the children and keep going
                children_list = self.get_children(current_node)
                for child in children_list:
                    self.add_to_fringe(child)

    def get_highest_priority_node(self):

        return Node()

    def hunter_in_open_set(self):
        pass

    def add_to_fringe(self, node):
        """
        Takes a node and adds it to the fringe (unexplored nodes)
        """

        if not self.fringe:
            # if there's nothing in the fringe, add the node
            self.fringe.append(node)
        elif node.f_cost > self.fringe[-1].f_cost:
            # if the node's cost is higher than everything else, just add it
            self.fringe.append(node)
        else:
            ind = 0
            while ind < len(self.fringe):
                if node.f_cost < self.fringe[ind].f_cost:
                    self.fringe.insert(ind, node)
                    return
                ind += 1
        pass

    def get_best_node_on_fringe(self):
        return self.fringe.pop(0)

    def get_children(self, node_to_expand: Node):

        parent_state = node_to_expand.state
        children_list = []
        for i in range(1, 3):
            for delta in [self.DISTANCE_DELTA, -self.DISTANCE_DELTA]:
                child_state = copy.deepcopy(parent_state)
                child_state[i] = child_state[i] + delta

                traj, traj_distance = construct_dubins_traj(parent_state, child_state)
                if not collision_found(traj, self.objects, self.walls) and child.state[1:4] != child.parent_node.state[
                                                                                               1:4]:
                    if self.grid[int(child.state[1]), int(child.state[2])] != 0:
                        child = Node(child_state, node_to_expand, self.desired_state, node_to_expand.tree)
                    else:
                        child = self.grid[int(child.state[1]), int(child.state[2])]

                    self.grid[int(child.state[1]), int(child.state[2])] = child
                    children_list.append(child)

        return children_list

    def generate_goal_node(self, node: Node, desired_state):
        """
         Another recommendation is to calculate a node's connection to the desired state with a generate_goal_node function
         that calculates a dubins traj between a node and the desired state. If this traj is collision free, the goal node
         should be created and returned so that the full trajectory can be built and returned. Note that you should only
         check for connections with a goal AFTER popping a node from the fringe (to ensure a form of optimality).
        """
        return self.create_node(desired_state, node)

    def create_node(self, state, parent_node: Node):

        # c(parent,n)
        c = self.calculate_edge_distance(state, parent_node)
        # g(n) is g(parent) + c(parent,n)
        g_cost = parent_node.g_cost + c
        # h is calculated as normal
        h_cost = self.estimate_cost_to_goal(state)

        return Node(state, parent_node, g_cost, h_cost)

    def create_initial_node(self, state):

        # it costs nothing to get to the start
        g_cost = 0
        # h is calculated as normal
        h_cost = self.estimate_cost_to_goal(state)

        return Node(state, None, g_cost, h_cost)

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

    def estimate_cost_to_goal(self, state):
        return math.sqrt((self.desired_state[1] - state[1]) ** 2 + (self.desired_state[2] - state[2]) ** 2)

    def build_traj(self, goal_node):

        node_list = []
        node_to_add = goal_node
        while node_to_add != None:
            node_list.insert(0, node_to_add)
            node_to_add = node_to_add.parent_node

        traj = []
        parent_time = None
        for i in range(1, len(node_list)):
            node_A = node_list[i - 1]
            node_B = node_list[i]
            traj_point_0 = node_A.state
            traj_point_1 = node_B.state
            traj_point_1[3] = math.atan2(traj_point_1[2] - traj_point_0[2], traj_point_1[1] - traj_point_0[1])
            if len(traj) > 0:
                parent_time = traj[-1][0]
            edge_traj, edge_traj_distance = construct_dubins_traj(traj_point_0, traj_point_1, parent_time=parent_time)
            traj = traj + edge_traj
        # print("TEST:", edge_traj)
        return traj

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

    def expand_nodes(self, node_to_expand: Node):

        pass

    def get_roots(self):
        pass

    def build_subtrees(self):
        pass


if __name__ == '__main__':
    for i in range(0, 5):
        maxR = 10
        tp0 = [0, -8, -8, 0]
        tp1 = [300, random.uniform(-maxR + 1, maxR - 1), random.uniform(-maxR + 1, maxR - 1), 0]
        planner = MTPP()
        walls = [[-maxR, maxR, maxR, maxR, 2 * maxR], [maxR, maxR, maxR, -maxR, 2 * maxR],
                 [maxR, -maxR, -maxR, -maxR, 2 * maxR], [-maxR, -maxR, -maxR, maxR, 2 * maxR]]
        num_objects = 25
        grid_size = 10
        objects = []
        for j in range(0, num_objects):
            obj = [random.uniform(-maxR + 1, maxR - 1), random.uniform(-maxR + 1, maxR - 1), 0.5]
            while (abs(obj[0] - tp0[1]) < 1 and abs(obj[1] - tp0[2]) < 1) or (
                    abs(obj[0] - tp1[1]) < 1 and abs(obj[1] - tp1[2]) < 1):
                obj = [random.uniform(-maxR + 1, maxR - 1), random.uniform(-maxR + 1, maxR - 1), 0.5]
            objects.append(obj)
        traj = planner.construct_traj(tp0, tp1, objects, walls, grid_size)
        if len(traj) > 0:
            plot_traj(traj, traj, objects, walls)
