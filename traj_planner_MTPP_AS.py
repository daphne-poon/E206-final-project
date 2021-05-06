# E206 Motion Planning

# Simple planner
# C Clark
import copy
import math
import dubins
import random
import operator
import matplotlib.pyplot as plt
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
        self.child_list = []
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
            self.g_cost = parent_node.g_cost + 1#parent_node.manhattan_distance_to_state(self.state)
            parent_node.child_list.append(self)
            # print("ADDED To CHILD LIST", parent_node.child_list)
        else:

            # print("NOTHING ADDED TO CHILD LIST!!!")
            self.g_cost = 1
        self.h_cost = self.euclidean_distance_to_state(chaser_state)
        self.f_cost = self.g_cost + self.h_cost
        self.tree = tree
        self.in_tree = True
    


class MTPP:
    DIST_TO_GOAL_THRESHOLD = 0.5  # m
    CHILDREN_DELTAS = [-0.5, -0.25, 0.0, 0.25, 0.5]
    DISTANCE_DELTA = 1  # m
    EDGE_TIME = 2  # s
    LARGE_NUMBER = 9999999
    N = 10

    def __init__(self):
        self.grid = self.contruct_grid()

    def contruct_grid(self):
        '''
        grid: empty NxN array of uninitialized nodes
        '''
        grid = [[Node([0, i, j]) for i in range(self.N)] for j in range(self.N)]
        # grid = np.zeros((self.N, self.N))
        # for i in range(self.N):
        #     for j in range(self.N):
        #         state = [0, i, j]
        #         grid[i][j] = Node(state) #TODO: change to list comp
    
        return grid

    def initial_path_finding(self, chaser_state, evader_state):
        """ Construct a trajectory in the X-Y space and in the time-X,Y,Theta space.
            Arguments:
            Returns:
              traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
        """
        # print(chaser_state)
        self.chaser_state = chaser_state
        self.evader_state = evader_state
        self.tree_roots = []
        self.old_tree_roots = []
        self.open_set = []
        self.leaf_set = []
        self.path_found = False

        # initial path finding
        self.tree_count = 0
        node_list = self.get_available_nodes(evader_state)

        for node in node_list:
            if node.type == 'chaser': #TODO: fix edgecase
                print("Chaser generated next to evader")
                node.update_node(None, chaser_state, self.tree_count)
                self.open_set.clear()
                return self.build_traj(node, self.grid[int(evader_state[2])][int(evader_state[1])])

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
                return self.build_traj(node, self.grid[int(evader_state[2])][int(evader_state[1])])


    def get_available_nodes(self, state_to_expand):
        '''
        returns list of nodes avalible for expansion
        '''
        node_list = []
        for i in [1, 2]:
            for delta in [self.DISTANCE_DELTA, -self.DISTANCE_DELTA]:
                state = copy.deepcopy(state_to_expand)
                if state[i] == 0 or state[i] == self.N:
                    continue
                state[i] = state[i] + delta
                # checks for wall
                if state[i]<self.N:
                    node = self.grid[int(state[2])][int(state[1])]
                    if node.type != 'obstacle':
                        node_list.append(node)
        return node_list
    
    def get_highest_priority_node(self):
        '''
        "pops" a node from the open set based on priority
        '''
        list_min = min(self.open_set, key=operator.attrgetter('f_cost')) # TODO: check if we should use f_cost instead
        priority_node = copy.deepcopy(list_min)
        self.open_set.remove(list_min) 
        return priority_node 
    
    def expand_node(self, node_to_expand):
        '''
        expands node by adding appropriate nodes to the open and leaf set
        '''
        node_list = self.get_available_nodes(node_to_expand.state)
        for node in node_list:
            temp_cost = node_to_expand.g_cost + 1 + node.euclidean_distance_to_state(self.chaser_state) #node_to_expand.manhattan_distance_to_state(node.state)
            
            # print("roots are")
            # for root in self.old_tree_roots:
            #     print(root.state)
            # print("node is", node.state)
            if node.type == 'chaser':
                print("found chaser at", node.state)
                node.update_node(node_to_expand, self.chaser_state, self.tree_count)
                self.path_found = True
                return node
            elif node.f_cost > temp_cost or node in self.old_tree_roots:
                if node.type != 'evader':
                    node.set_empty()
                    if node.parent_node != None:
                        node.parent_node.child_list.remove(node)
                node.update_node(node_to_expand, self.chaser_state, self.tree_count)
                self.open_set.append(node)
                if node in (self.leaf_set):
                    self.leaf_set.remove(node)
            if node.f_cost < temp_cost and node.tree!=node_to_expand.tree:
                self.leaf_set.append(node)
                if node_to_expand not in self.leaf_set:
                    self.leaf_set.append(node_to_expand)


    def build_traj(self, goal_node, evader_node):

        node_list = []
        node_to_add = goal_node
        while node_to_add != None:
            print(f'x: {node_to_add.state[1]}, y: {node_to_add.state[2]}, fcost: {node_to_add.f_cost}, gcost: {node_to_add.g_cost}, hcost: {node_to_add.h_cost}')
            node_list.insert(0, node_to_add)
            node_to_add = node_to_add.parent_node

        node_list.insert(0, evader_node)

        node_list.reverse()

        traj = []
        parent_time = None
        traj_point_0 = [0, 0, 0, 0]
        traj_point_1 = [0, 0, 0, 0]
        discretized_traj = []
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
            discretized_traj += [node_list[i].state]
            traj = traj + edge_traj

        return traj, discretized_traj

    def build_disctrtized_traj(self, goal_node, evader_node):
        node_list = []
        node_to_add = goal_node
        while node_to_add != None:
            print(f'x: {node_to_add.state[1]}, y: {node_to_add.state[2]}, fcost: {node_to_add.f_cost}, gcost: {node_to_add.g_cost}, hcost: {node_to_add.h_cost}')
            node_list.insert(0, node_to_add)
            node_to_add = node_to_add.parent_node

        node_list.insert(0, evader_node)

        node_list.reverse()

        discretized_traj = []
        for i in range(1, len(node_list)):
            discretized_traj += [node_list[i].state]
        
        discretized_traj += evader_node.state
        
        return traj, discretized_traj
    
    def traverse(self, chaser, evader):
        self.rendezvous = False

        timesteps  = 0

        while chaser.node.parent_node is not None:
            chaser.update(chaser.node.parent_node)
            #TODO: change criterion later
            if timesteps < 5:
                self.path_found = False
                evader = self.update_evader(evader)
                timesteps += 1
                # CHECK: deep copy?
                self.old_tree_roots = copy.deepcopy(self.tree_roots)
                self.tree_roots = []
                
                self.tree_count = 0
                node_list = self.get_available_nodes(evader_state)
                
                for node in node_list:
                    #TODO: two evaders should not be able to be at the same cell
                    if node.type == 'uninitialized':
                        node.set_empty()
                    node.update_node(None, chaser_state, self.tree_count)
                    self.tree_roots.append(node)
                    self.open_set.append(node)
                    self.tree_count += 1
                
                # print("old_tree_roots", self.old_tree_roots)
                while not all([x in self.open_set for x in self.old_tree_roots]):
                    # print("FINDING OLD ROOTS")
                    # print("open set", self.open_set)
                    if len(self.open_set) == 0:
                        print("Error: Paths to old roots not found")
                    priority_node = self.get_highest_priority_node()
                    self.tree_count = priority_node.tree
                    node = self.expand_node(priority_node)
                
                for root in self.old_tree_roots:
                    self.update_tree_cost(root)

                # print("0 LEAF SET: ", self.leaf_set, "OPEN SET: ", self.open_set)
                self.open_set = copy.deepcopy(self.leaf_set)
                self.leaf_set = []
                # print("1 LEAF SET: ", self.leaf_set, "OPEN SET: ", self.open_set)

                while True:
                    if len(self.open_set) == 0:
                        print("Error: Corrected path not found")
                    priority_node = self.get_highest_priority_node()
                    self.tree_count = priority_node.tree
                    node = self.expand_node(priority_node)
                    if self.path_found:
                        self.open_set.clear()
                        path, discretized_path = self.build_traj(node, self.grid[int(evader_state[2])][int(evader_state[1])])
                        break  
        return chaser.traj, evader.traj


    def update_evader(self, evader):
        '''
        determines how evader moves
        '''
        node_to_expand = evader.traj[-1]
        node_list = self.get_available_nodes(node_to_expand.state)

        for node in node_list:
            if node.state[1]<node_to_expand.state[1]:
                evader.update(node)
        return evader
    
    def update_tree_cost(self, root):
        # print("ITS A TREE")
        node_list = []
        nodes_to_add = [root]
        # print(root.child_list)
        while len(nodes_to_add) != 0:
            temp_node = nodes_to_add.pop(0)
            node_list.append(temp_node)
            nodes_to_add += temp_node.child_list

        for node in node_list:
            # print(node.state)
            node.g_cost += root.g_cost
            node.f_cost += root.g_cost


class Chaser:
    
    def __init__(self, node):
        self.node = node
        self.traj = [node]
    
    def update(self, node):
        self.traj[-1].set_empty()
        node.set_chaser()
        self.node = node
        self.traj.append(node)


class Evader:
    
    def __init__(self, node):
        self.node = node
        self.traj = [node]
    
    def update(self, node):
        self.traj[-1].set_empty()
        node.set_evader()
        self.node = node
        self.traj.append(node)



if __name__ == '__main__':
    # for i in range(0, 5):
    # initialize planner
    planner = MTPP()
    # initialize chaser
    chaser_state = [0, 2, 5]
    planner.grid[chaser_state[2]][chaser_state[1]].set_chaser()
    chaser = Chaser(planner.grid[chaser_state[2]][chaser_state[1]])
    # initialize evaders
    evaders = [] #TODO: implement multiple
    evader_state = [0, 8, 1]
    planner.grid[evader_state[2]][evader_state[1]].set_evader()
    evader = Evader(planner.grid[evader_state[2]][evader_state[1]])
    evaders.append(evader)
    # tp1 = [300, random.uniform(-maxR + 1, maxR - 1), random.uniform(-maxR + 1, maxR - 1), 0]

    # initial path finding
    initial_path, discretized_path = planner.initial_path_finding(chaser_state, evader_state)

    maxR = 10
    walls = [[-maxR, maxR, maxR, maxR, 2 * maxR], [maxR, maxR, maxR, -maxR, 2 * maxR],
                [maxR, -maxR, -maxR, -maxR, 2 * maxR], [-maxR, -maxR, -maxR, maxR, 2 * maxR]]
    
    objects = []
    # plot_traj(initial_path, initial_path, objects, walls)
    
    # begin moving robot
    chaser_path, evader_path = planner.traverse(chaser, evader)
    plot_traj(initial_path, initial_path, objects, walls)


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
