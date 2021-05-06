import math
import numpy as np
import matplotlib.pyplot as plt
from APF_utilities import *
import math
import matplotlib.pyplot as plt

class APF_Planner():

  def __init__(self, map):
    self.sync_to_map(map)
  
  def sync_to_map(self, map):
    self.map = map
    self.evaders = []
    for evader_state in self.map.evader_states:
        self.evaders.append(map[evader_state[1]][evader_state[2]])

  def update_evaders(self, map):
    self.sync_to_map(map)
    new_evader_states = []
    for evader_state in self.map.evader_states:
      force_vector = self.generate_force_vector(self.map.chaser_state, evader_state)
      if force_vector == [0, 0]:
        print("no force on evader")
      new_evader_state = self.get_update_state(force_vector, evader_state)
      new_evader_states.append(new_evader_state)
      #update grid
      self.map.grid[evader_state[1]][evader_state[2]].set_empty()
      self.map.grid[new_evader_state[1]][new_evader_state[2]].set_evader()
    
    self.map.evader_states = new_evader_states

    return self.map
  
  def generate_force_vector(self, chaser_state, evader_state):
    # Add attraction to goal
    desired_state = self.get_desired_corner(chaser_state, evader_state)
    force = self.attractionForce(desired_state, evader_state)
    # Add repulsion from chaser
    force = self.add(force, self.repulsionForce(evader_state, chaser_state))
    return self.normalize(force)

  def get_desired_corner(self, chaser_state):
    corners = [[0, 0, 0], [0, self.map.N-1, 0], [0, 0, self.map.N-1], [0, self.map.N-1, self.map.N-1]]
    distances = []
    for corner in corners:
      distances.append(self.map.grid[chaser_state[1]][chaser_state[2]].euclidean_distance_to_state(corner))
    return corners[distances.index(min(distances))]

  def attractionForce(self, desired_state, evader_state):
    k_att = 1
    x_att = k_att * (desired_state[1] - evader_state[1])
    y_att = k_att * (desired_state[1] - evader_state[1])
    return [x_att, y_att]
  
  def repulsionForce(self, evader_state, chaser_state):
    k_rep = 200
    rho_0 = 10
    rho_p = math.sqrt((evader_state[1] - chaser_state[1])**2 + (evader_state[2] - chaser_state[2])**2)
    x_rep = k_rep*((1/(rho_p))-(1/rho_0))*((evader_state[1] - chaser_state[1])/(rho_p))
    y_rep = k_rep*((1/(rho_p))-(1/rho_0))*((evader_state[2] - chaser_state[2])/(rho_p))
    return [x_rep, y_rep]
        
  def add(self, force_1, force_2):
    return [force_1[0] + force_2[0], force_1[1] + force_2[1]]
      
  def normalize(self, vector):
    length = math.sqrt(vector[0]**2 + vector[1]**2)
    return [vector[0] / length, vector[1] / length]

  def get_update_state(self, force_vector, evader_state):
    '''
    gets the cell that the evader should travel to
    '''
    update_list = [[0, evader_state[1] + np.sign(force_vector[1]), evader_state[2]],
                  [0, evader_state[1], evader_state[2] + np.sign(force_vector[2])] ]
    
    if abs(force_vector[0])<abs(force_vector[1]):
      update_list.reverse()

    update_list.append(evader_state)

    for update_state in update_list:
      current_node = self.map.grid[update_state[1]][update_state[2]]
      if update_state== evader_state or current_node.type == 'empty':
        return update_state

    print("Error: no update state returned for evader")



if __name__ == '__main__':
    # for i in range(0, 5):
    # initialize planner
    planner = APFplanner()