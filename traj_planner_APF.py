import math
import numpy as np
import matplotlib.pyplot as plt
import math
import matplotlib.pyplot as plt
import random

class APF_Planner():

  def __init__(self, map):
    self.sync_to_map(map)
  
  def sync_to_map(self, map):
    self.map = map
    self.evaders = []
    for evader_state in self.map.evader_states:
        self.evaders.append(map.grid[evader_state[1]][evader_state[2]])

  def update_evader_positions(self, map):
    self.sync_to_map(map)
    new_evader_states = []
    for evader_state in self.map.evader_states:

      if self.map.evader_states.index(evader_state) in self.map.dead_evaders:
        break

      force_vector = self.generate_force_vector(self.map.chaser_state, evader_state)
      if force_vector == [0, 0]:
        print("no force on evader")
      new_evader_state = self.get_update_state(force_vector, evader_state)
      new_evader_states.append(new_evader_state)
      #update grid
      print('old state:', evader_state, "new state:", new_evader_state)
      self.map.grid[evader_state[1]][evader_state[2]].set_empty()
      self.map.grid[new_evader_state[1]][new_evader_state[2]].set_evader()
    
    self.map.evader_states = new_evader_states

    return self.map.evader_states
  
  def generate_force_vector(self, chaser_state, evader_state):
    # Add attraction to goal
    desired_state = self.get_desired_corner(chaser_state)
    print("CORNER:", desired_state, chaser_state)
    force = self.attractionForce(desired_state, evader_state)
    # Add repulsion from chaser
    force = self.add(force, self.repulsionForce(evader_state, chaser_state))
    if force != [0, 0]:
      return self.normalize(force)
    return force

  def get_desired_corner(self, chaser_state):
    corners = [[0, 0, 0], [0, self.map.N-1, 0], [0, 0, self.map.N-1], [0, self.map.N-1, self.map.N-1]]
    distances = []
    for corner in corners:
      distances.append(self.map.grid[chaser_state[1]][chaser_state[2]].euclidean_distance_to_state(corner))
    return corners[distances.index(max(distances))]

  def attractionForce(self, desired_state, evader_state):
    k_att = 5
    x_att = k_att * (desired_state[1] - evader_state[1])
    y_att = k_att * (desired_state[1] - evader_state[1])
    print("ATTRACTION FORCE:", [x_att, y_att])
    return [x_att, y_att]
  
  def repulsionForce(self, evader_state, chaser_state):
    k_rep = 200
    rho_0 = 10
    rho_p = math.sqrt((evader_state[1] - chaser_state[1])**2 + (evader_state[2] - chaser_state[2])**2)
    x_rep = k_rep*((1/(rho_p))-(1/rho_0))*((evader_state[1] - chaser_state[1])/(rho_p))
    y_rep = k_rep*((1/(rho_p))-(1/rho_0))*((evader_state[2] - chaser_state[2])/(rho_p))
    print("REPULSION FORCE:", [x_rep, y_rep])
    return [x_rep, y_rep]
        
  def add(self, force_1, force_2):
    print("ADDED:", [force_1[0] + force_2[0], force_1[1] + force_2[1]])
    return [force_1[0] + force_2[0], force_1[1] + force_2[1]]
      
  def normalize(self, vector):
    length = math.sqrt(vector[0]**2 + vector[1]**2)
    return [vector[0] / length, vector[1] / length]

  def get_update_state(self, force_vector, evader_state):
    '''
    gets the cell that the evader should travel to
    '''
    print("Force vectors", force_vector)
    if force_vector[0] == 0:
      force_vector[0] = random.choice([force_vector[1] / 2, -force_vector[1] / 2])
    elif force_vector[1] == 0:
      force_vector[1] = random.choice([force_vector[0] / 2, -force_vector[0] / 2])

    update_list = []
    if int(evader_state[1] + np.sign(force_vector[0])) >= 0 and int(evader_state[1] + np.sign(force_vector[0])) < self.map.N:
      update_list.append([0, int(evader_state[1] + np.sign(force_vector[0])), evader_state[2]])
    if int(evader_state[2] + np.sign(force_vector[1])) >= 0 and int(evader_state[2] + np.sign(force_vector[1])) < self.map.N:
      update_list.append([0, evader_state[1], int(evader_state[2] + np.sign(force_vector[1]))])
    
    if abs(force_vector[0])<abs(force_vector[1]):
      update_list.reverse()

    update_list.append(evader_state)
    print("update list", update_list)

    for update_state in update_list:
      current_node = self.map.grid[update_state[1]][update_state[2]]
      print("state:", update_state, "is of type:", current_node.type)
      if update_state == evader_state or current_node.type == 'empty' or current_node.type == 'uninitialized':
        return update_state

    print("Error: no update state returned for evader")



if __name__ == '__main__':
    # for i in range(0, 5):
    # initialize planner
    planner = APFplanner()