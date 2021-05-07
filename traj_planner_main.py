# E206 Motion Planning

# Combined trajectory planner (evader and chaser)
# Daphne Poon and Sabrina Shen
import functools
import random
import time

from map import Map
from traj_planner_APF import APF_Planner
from traj_planner_MTPP import MTPP


def timer(func):
    @functools.wraps(func)
    def wrapper_timer(*args, **kwargs):
        tic = time.perf_counter()
        value = func(*args, **kwargs)
        toc = time.perf_counter()
        elapsed_time = toc - tic
        print(f"Elapsed time: {elapsed_time:0.4f} seconds")
        return value

    return wrapper_timer


class Environment():
    DIST_TO_GOAL_THRESHOLD = 0.5  # m
    # N = 20
    # num_evaders = 3  # ez
    GOAL_SWITCH_THRESHOLD = 0.5  # as a ratio of distance
    LARGE_NUMBER = 9999999

    def __init__(self, chaser_state, evader_states, num_evaders, N):
        self.N = N
        self.num_evaders = num_evaders
        self.map = Map(self.N, self.num_evaders, chaser_state, evader_states)
        self.chaser_planner = MTPP(self.map)
        self.evader_planner = APF_Planner(self.map)


    def target_is_reached(self):
        evader_node = self.map.get_evader_node(self.map.current_evader)
        if evader_node.euclidean_distance_to_state(self.map.chaser_state) <= 1:
            self.map.chaser_state = self.chaser_planner.update_chaser_position(evader_node)
            return True
        return False

    @timer
    def build_dynamic_path_to_targets(self):
        self.map.show_current_grid()
        while self.num_evaders > 0:

            self.reset_maps()

            # get the closest node to chase
            self.map.current_evader = self.get_closest_evader()
            _, discretized_path = self.chaser_planner.initial_path_finding()
            self.sync_maps(from_chaser=True)

            self.map.show_current_grid(tree=True)
            self.map.show_current_grid(traj = discretized_path, tree=False)

            if self.target_is_reached():
                self.sync_maps(from_chaser=True)
                # print(f"CAUGHT evader {self.map.current_evader}")
                self.map.evader_states.pop(self.map.current_evader)
                self.num_evaders = self.num_evaders - 1
                if len(self.map.dead_evaders) == self.num_evaders:
                    break
            else:
                # check if we need to go to a new evader...
                update_evader, new_id = self.update_evader()
                if update_evader:
                    # self.map.show_current_grid()
                    # print(f"turns out evader {new_id} is closer!")
                    self.map.current_evader = new_id
                    self.reset_maps()
                    self.chaser_planner = MTPP(self.map)
                    _, discretized_path = self.chaser_planner.initial_path_finding()
                    self.sync_maps(from_chaser=True)
                else:
                    # print(f"tracking same evader as before: {self.map.current_evader}")
                    self.map.chaser_state = self.chaser_planner.update_chaser_position()
                    self.sync_maps(from_chaser=True)
                    self.map.evader_states = self.evader_planner.update_evader_positions()
                    self.sync_maps(from_evader=True)
                    _, discretized_path = self.chaser_planner.correct_path()
                    self.sync_maps(from_chaser=True)

            # self.map.show_current_grid(discretized_path)
            # self.map.show_current_grid(tree=True)

        # self.map.show_current_grid(discretized_path)
        # return self.map.discretized

    def sync_maps(self, from_chaser=False, from_evader=False):
        if from_chaser:
            self.map = self.chaser_planner.map
            self.evader_planner.map = self.chaser_planner.map
        elif from_evader:
            pass
            self.map = self.evader_planner.map
            self.chaser_planner.map = self.evader_planner.map
        else:
            self.chaser_planner.map = self.map
            self.evader_planner.map = self.map

    def update_evader(self):
        chaser_node = self.map.get_chaser_node()
        current_evader = self.map.get_evader_node(self.map.current_evader)
        dist_to_curr_node = chaser_node.euclidean_distance_to_state(current_evader.state)
        alive_evaders = [x for x in range(self.num_evaders) if x not in self.map.dead_evaders]
        for node_id in alive_evaders:
            evader_node = self.map.get_evader_node(node_id)
            dist_to_node = chaser_node.euclidean_distance_to_state(evader_node.state)
            if dist_to_node <= (self.GOAL_SWITCH_THRESHOLD * dist_to_curr_node):
                return True, node_id
        return False, 0

    def get_closest_evader(self):
        chaser_node = self.map.get_chaser_node()
        min_dist = self.LARGE_NUMBER
        min_id = 0

        for node_id in range(self.num_evaders):
            evader_node = self.map.get_evader_node(node_id)
            dist_to_node = chaser_node.euclidean_distance_to_state(evader_node.state)
            if dist_to_node < min_dist and (node_id not in self.map.dead_evaders):
                min_id = node_id
                min_dist = dist_to_node

        return min_id

    def reset_maps(self):
        chaser_state_new = self.map.chaser_state
        evader_states_new = self.map.evader_states
        obstacles = self.map.obstacles
        # make a new map
        self.map = Map(self.N, self.num_evaders, chaser_state_new, evader_states_new)

        # set the chaser
        self.map.grid[chaser_state_new[1]][chaser_state_new[2]].set_chaser()
        # set the evader
        for evader_state in evader_states_new:
            self.map.grid[evader_state[1]][evader_state[2]].set_evader()
        # set obstacles
        self.map.add_obstacles(obstacles)

        # sync em up
        self.sync_maps()


if __name__ == '__main__':
    # init environment
    N = 20

    # environment = Environment(chaser_state, evader_states, 3, 10)
    for _ in range(20):
        chaser_state = [0, 1, 2]
        evader_states = []
        for _ in range(3):
            rand_x = random.randint(0, N - 1)
            rand_y = random.randint(0, N - 1)
            if [rand_x, rand_y] != [1, 2]:
                evader_states.append([0, rand_x, rand_y])
        # print(f'evader states {evader_states}')
        # evader_states = [[0, 12, 3], [0, 6, 5], [0, 5, 8]]
        environment = Environment(chaser_state, evader_states, 3, N)
        # initialize obstacles
        random.seed(time.time())
        obstacles = []
        for _ in range(int(0.2 * environment.N ** 2)):
            rand_x = random.randint(0, environment.N - 1)
            rand_y = random.randint(0, environment.N - 1)
            if environment.map.grid[rand_x][rand_y].type == 'uninitialized':
                obstacles.append([rand_x, rand_y])
        environment.map.add_obstacles(obstacles)

        # initialize chaser and evaders
        environment.map.grid[chaser_state[1]][chaser_state[2]].set_chaser()
        for evader_state in evader_states:
            environment.map.grid[evader_state[1]][evader_state[2]].set_evader()

        environment.build_dynamic_path_to_targets()
