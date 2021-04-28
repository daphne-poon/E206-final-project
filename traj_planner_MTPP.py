# E206 Motion Planning

# Moving Target Path Planner (chaser)
# Daphne Poon and Sabrina Shen

from traj_planner_utils import *


class Node():

    def __init__(self, state, parent_node, tree, cost, priority, neighbors):
        self.state = state
        self.parent_node = parent_node
        self.tree = tree
        self.cost = cost
        self.priority = priority
        self.neighbors = neighbors

    def manhattan_distance_to_node(self, node):
        return abs(self.state[1] - node.state[1]) + abs(self.state[2] - node.state[2])

    def manhattan_distance_to_state(self, state):
        return abs(self.state[1] - state[1]) + abs(self.state[2] - state[2])

    def euclidean_distance_to_state(self, state):
        return math.sqrt((self.state[1] - state[1]) ** 2 + (self.state[2] - state[2]) ** 2)


class Moving_Target_Path_Planner():
    BIG_NUMBER = 100000

    def __init__(self):
        pass

    def construct_traj(self, initial_state, desired_state, objects, walls):
        """ Construct a trajectory in the X-Y space and in the time-X,Y,Theta space.
            Arguments:
              initial_state (list of floats): The trajectory's initial state with time, X, Y, Theta (s, m, m, rad).
              desired_state (list of floats): The trajectory's desired state with time, X, Y, Theta (s, m, m, rad).
              objects (list of lists): A list of object states - X, Y, radius (m, m, m).
              walls (list of lists): A list of walls defined by end points - X0, Y0, X1, Y1, length (m, m, m, m, m).
            Returns:
              traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
              traj_cost (float): The path length (m).
        """

        # initialize run
        traj = []
        traj_cost = self.BIG_NUMBER
        self.leafset = []
        self.openset = []
        self.closedset = []

        # initial path finding

        # navigate along path
        while not self.goal_reached():
            self.update_robot_state()
            self.update_goal_state()
            # correct path, if necessary
            if self.current_goal_state != self.previous_goal_state:
                self.correct_path()

        return traj, traj_cost

    def goal_reached(self):
        pass

    def update_robot_state(self):
        pass

    def update_goal_state(self):
        """ Updates the goal state of a given node.
        """
        pass

    def expand_node(self, current_node):
        """ Expands all neighboring nodes of given node.
            Arguments:
              current_node (Node): The current node to expand.
        """

        for neighbor in current_node.neighbors:
            cost_through_current_node = current_node.cost + current_node.manhattan_distance_to_node(neighbor)
            if neighbor.cost > cost_through_current_node:
                neighbor.cost = cost_through_current_node
                neighbor.tree = current_node.tree
                neighbor.parent = current_node
                self.openset.append(neighbor)
                if neighbor in self.leafset:
                    self.leafset.remove(neighbor)
            if (neighbor.cost < cost_through_current_node) and (neighbor.tree != current_node.tree):
                self.add_to_leafset([neighbor, current_node])

    def add_to_leafset(self, nodes):
        """ Adds a list of nodes to the leafset (unless they're already in the leafset).
            Arguments:
              nodes (List of Nodes): The nodes to add.
        """

        for node in nodes:
            if node not in self.leafset:
                self.leafset.append(node)
