# E206 Motion Planning

# Simple planner
# C Clark
import copy
import operator
import random

from traj_planner_utils import *


class MTPP:
    DISTANCE_DELTA = 1  # m
    LARGE_NUMBER = 9999999

    def __init__(self, initial_map):
        self.map = initial_map

        # MTPP specific
        self.tree_roots = []
        self.old_tree_roots = []
        self.leaf_set = []
        self.open_set = []
        self.closed_set = []
        self.current_tree = 0

    def initial_path_finding(self):
        """ Construct a trajectory in the X-Y space and in the time-X,Y,Theta space.
            Arguments:
              chaser_state (list): state of the chaser (time, X, Y).
              evader_state (list): state of the evader (time, X, Y).
            Returns:
              traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
              discretized_traj (list of lists): A list of path points with time, X, Y (s, m, m).
        """

        # Reinitialize run
        self.current_tree = 0
        self.open_set.clear()
        self.closed_set.clear()
        self.leaf_set.clear()
        self.tree_roots.clear()
        self.old_tree_roots.clear()
        # for level in self.map.grid:
        #     for node in level:
        #         node.parent_node = None
        #         node.children_nodes = []
        #         node.tree = None
        #         node.in_tree = False
                # if node.type != 'chaser':
                #     node.g_cost = self.LARGE_NUMBER
                #     node.h_cost = self.LARGE_NUMBER
                #     node.f_cost = self.LARGE_NUMBER


        # Lines 3-12
        node_list = self.get_neighboring_nodes(self.map.evader_states[self.map.current_evader])
        # print(f'node list is {node_list}')
        for node in node_list:
            if node.type == 'uninitialized':
                node.set_empty()
            node.update_node(None, self.map.chaser_state, self.current_tree)
            self.tree_roots.append(node)
            self.open_set.append(node)
            # print(f'node is appeneded to open set {node}')
            self.current_tree += 1

        # Lines 13-18
        # print(f'oepn set is {self.open_set}')
        while not self.chaser_in_open_set():
            if len(self.open_set) == 0:
                # print("Error: No path exists")
                break
            priority_node = self.get_highest_priority_node()
            if priority_node.state[1:] == self.map.chaser_state[1:]:
                break
            self.expand_node(priority_node)

        # Line 19
        for open_node in self.open_set:
            self.closed_set.append(open_node)

        # Line 20
        self.open_set.clear()

        # Line 21
        return self.build_traj_from_goal()

    def get_neighboring_nodes(self, state_to_expand):
        """ Returns all neighboring nodes of an input node, provided they are not obstacles.
            Arguments:
              state_to_expand (list): state of the node being expanded (time, X, Y).
            Returns:
              node_list (list of Nodes): A list of nodes we can expand to.
        """
        node_list = []
        for i in [1, 2]:
            for delta in [self.DISTANCE_DELTA, -self.DISTANCE_DELTA]:
                state = copy.deepcopy(state_to_expand)
                state[i] = state[i] + delta

                # checks for walls
                if (state[i] >= self.map.N) or (state[i] < 0):
                    continue
                node = self.map.grid[int(state[1])][int(state[2])]

                if (not node.is_obstacle()) and (node.state[1:] != self.map.evader_states[self.map.current_evader][1:]):
                    node_list.append(node)
        return node_list

    def get_highest_priority_node(self):
        '''
        "pops" a node from the open set based on priority
        '''
        list_min = min(self.open_set, key=operator.attrgetter('h_cost'))
        return list_min

    def expand_node(self, node_to_expand):
        '''
        expands node by adding appropriate nodes to the open and leaf set
        '''
        node_list = self.get_neighboring_nodes(node_to_expand.state)
        for node in node_list:
            temp_cost = node_to_expand.g_cost + 1

            # Lines 3-12
            if (node.g_cost > temp_cost) and (node_to_expand.parent_node != node):
                if node.type == 'uninitialized':
                    node.set_empty()
                old_parent = node.parent_node
                if (old_parent is not None) and (old_parent != node_to_expand):
                    old_parent.children_nodes.remove(node)
                node_to_expand.children_nodes.append(node)
                node.update_node(node_to_expand, self.map.chaser_state, node_to_expand.tree)
                self.open_set.append(node)
                if node.state in [x.state for x in self.old_tree_roots]:
                    # print(f'saw {node.state}!!')
                    self.seen_roots.append(node)
                if node in self.leaf_set:
                    self.leaf_set.remove(node)

            # Lines 13-17
            if node.g_cost < temp_cost and node.tree != node_to_expand.tree:
                if node not in self.leaf_set:
                    self.leaf_set.append(node)
                if node_to_expand not in self.leaf_set:
                    self.leaf_set.append(node_to_expand)

        # Lines 18-19
        self.closed_set.append(node_to_expand)
        self.open_set.remove(node_to_expand)

    def build_traj_from_goal(self):
        goal_node = self.map.grid[self.map.chaser_state[1]][self.map.chaser_state[2]]
        return self.build_traj(goal_node)

    def build_traj(self, goal_node):
        # print(f"goal node is {goal_node}")
        node_list = []
        node_to_add = goal_node
        while node_to_add != None:
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

        # print(f"traj is {traj}")
        # print(f"disc is {discretized_traj}")
        return traj, discretized_traj

    def chaser_in_open_set(self):
        chaser_node = self.map.get_chaser_node()
        return chaser_node in self.open_set

    def correct_path(self):
        self.closed_set.clear()
        self.open_set.clear()
        self.old_tree_roots.clear()

        # Line 3
        for root in self.tree_roots:
            self.old_tree_roots.append(root)

        # Line 4
        # print("RESETTING OLD ROOTS\n")
        for root_node in self.old_tree_roots:
            root_node.g_cost = self.LARGE_NUMBER
            root_node.h_cost = self.LARGE_NUMBER
            root_node.f_cost = self.LARGE_NUMBER

        # Line 5
        # self.current_tree = 0

        # Lines 6-15
        node_list = self.get_neighboring_nodes(self.map.evader_states[self.map.current_evader])

        self.tree_roots.clear()
        self.seen_roots = []
        rr = [x.state for x in self.old_tree_roots]
        # print(f'roots are: {rr}')
        # print("ADDING NEW ROOTS\n")
        for node in node_list:
            node.update_node(None, self.map.chaser_state, self.current_tree)

            if node.state in rr:
                # print(f'saw {node.state}!!')
                self.seen_roots.append(node)
            self.tree_roots.append(node)
            self.open_set.append(node)
            self.current_tree += 1

        # Lines 16-21
        # print("EXPANDING TO OLD ROOTS\n")

        while len(self.seen_roots) < len(self.old_tree_roots):
            # print(f"open list is {self.open_set}")
            if len(self.open_set) == 0:
                # print("open set is empty!!!!")
                break
            priority_node = self.get_highest_priority_node()
            self.expand_node(priority_node)

        # Lines 22-26
        # print("UPDATING COSTS\n")
        self.update_costs()

        # Line 27
        for open_node in self.open_set:
            self.closed_set.append(open_node)

        # Line 28
        self.open_set.clear()

        # Line 29
        for leaf in self.leaf_set:
            self.open_set.append(leaf)

        # Line 30
        self.leaf_set.clear()

        # print("LOOKING FOR CHASER NOW\n")
        # Lines 31-36
        if len(self.leaf_set) > 0:
            while not self.chaser_in_open_set():
                if len(self.open_set) == 0:
                    break
                priority_node = self.get_highest_priority_node()
                self.expand_node(priority_node)

        # Line 36
        return self.build_traj_from_goal()

    def update_evader_position(self, node_id=0):
        evader_node = self.map.get_evader_node(node_id)
        node_list = self.get_neighboring_nodes(self.map.evader_states[self.map.current_evader])

        if len(node_list) == 0:
            # print("Error: Evader is trapped!!!")
            raise RuntimeError
        else:
            new_node = random.choice(node_list)
            new_node.set_evader()
            evader_node.set_empty()
            evader_node.g_cost = self.LARGE_NUMBER
            evader_node.f_cost = self.LARGE_NUMBER
            evader_node.f_cost = self.LARGE_NUMBER

        return new_node.state

    def update_chaser_position(self):
        chaser_node = self.map.get_chaser_node()
        parent_node = chaser_node.parent_node

        parent_node.set_chaser()
        chaser_node.set_empty()

        return parent_node.state

    def old_roots_in_open_set(self):
        for root in self.old_tree_roots:
            if root not in self.open_set:
                return False

        return True

    def update_costs(self):
        for root in self.old_tree_roots:
            root_cost = root.g_cost
            nodes_to_update = []
            for node in root.children_nodes:
                nodes_to_update.append(node)
            nodes_updated = []
            while len(nodes_to_update) > 0:
                current_node = nodes_to_update[0]
                current_node.g_cost = current_node.g_cost + root_cost
                for child in current_node.children_nodes:
                    if (child not in nodes_updated) and (child not in nodes_to_update):
                        nodes_to_update.append(child)
                nodes_to_update.remove(current_node)
                nodes_updated.append(current_node)

    def update_evader_positions(self):
        new_evader_states = self.map.evader_states
        alive_evaders = [x for x in range(self.map.num_evaders) if x not in self.map.dead_evaders]
        for node_id in alive_evaders:
            evader_node = self.map.get_evader_node(node_id)
            node_list = self.get_neighboring_nodes(evader_node.state)

            if len(node_list) == 0:
                # print("Error: Evader is trapped!!!")
                return None
            else:
                new_node = random.choice(node_list)
                new_node.set_evader()
                evader_node.set_empty()
                evader_node.g_cost = self.LARGE_NUMBER
                evader_node.f_cost = self.LARGE_NUMBER
                evader_node.f_cost = self.LARGE_NUMBER

            new_evader_states[node_id] = new_node.state
        return new_evader_states
