# E206 Motion Planning
import math


class Node:
    LARGE_NUMBER = 9999999

    def __repr__(self):
        return f"[type: {self.type}, state: {self.state}, tree: {self.tree}, costs: {self.g_cost, self.h_cost}]"

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
        self.children_nodes = []
        self.g_cost = self.LARGE_NUMBER
        self.h_cost = self.LARGE_NUMBER
        self.f_cost = self.LARGE_NUMBER
        self.tree = None
        self.in_tree = False
        self.type = 'uninitialized'
        self.evader_id = None

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
        self.h_cost = 0
        self.f_cost = self.g_cost + self.h_cost

    def set_evader(self):
        self.type = 'evader'
        self.g_cost = 0
        self.f_cost = self.g_cost + self.h_cost

    def is_obstacle(self):
        return self.type == 'obstacle'

    def update_node(self, parent_node, chaser_state, tree):
        self.parent_node = parent_node
        if parent_node is not None:
            self.g_cost = parent_node.g_cost + 1
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

    def get_tree(self):
        if self.tree is None:
            return 0
        else:
            return self.tree + 1