# E206 Motion Planning

# Simple planner
# Daphne Poon and Sabrina Shen

from traj_planner_utils import *


class Node():

    def __init__(self, state, parent_node, edge_distance):
        self.state = state
        self.parent_node = parent_node
        self.edge_distance = edge_distance

    def manhattan_distance_to_node(self, node):
        return abs(self.state[1] - node.state[1]) + abs(self.state[2] - node.state[2])

    def manhattan_distance_to_state(self, state):
        return abs(self.state[1] - state[1]) + abs(self.state[2] - state[2])

    def euclidean_distance_to_state(self, state):
        return math.sqrt((self.state[1] - state[1]) ** 2 + (self.state[2] - state[2]) ** 2)


class Moving_Target_Path_Planner():

    def __init__(self):
        self.fringe = []

if __name__ == '__main__':
    pass
