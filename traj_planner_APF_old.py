import math
import matplotlib.pyplot as plt
from APF_utilities import *
import math
import matplotlib.pyplot as plt

default_evader_radius = 0.5 #m
initial_score = 1000
goal_threshold = 0.5 #m

class Pose():
  x = 0
  y = 0
  theta = 0
  
  def __init__(self, x, y, theta):
    self.x = x
    self.y = y
    self.theta = theta
    
  def distance_to(self, pose):
    return math.sqrt((self.x - pose.x)**2 + (self.y - pose.y)**2)

class APFplanner():
#   pose = Pose(0, 0, 0)
#   goal_pose = Pose(0,0,0)
#   v = 0
#   w = 0
#   id = -1
#   K_w = 1
#   K_v = 1
#   w_max = 1
#   v_max = 2
#   radius = 0.5
#   alive = True
#   af0 = evaderForce_0()
#   af1 = evaderForce_1()
#   af3 = evaderForce_3()

  def __init__(self, grid, num_evaders, evader_states, chaser_state):
    """ Constructor
    """
    self.N = N
    self.grid = grid
    self.chaser_state = chaser_state
    self.evader_states = evader_states
    self.num_evaders = num_evaders
    self.evaders = []
    for state in evader_states:
        self.evaders.append(grid[state[1]][state[2]])
    

    # self.goal_pose = goal_pose
    # self.radius = radius
    # self.id = id
    # self.v = 0
    # self.w = 0
    # self.score = 0
    # self.collided = False
    return
    
  def update(self, delta_t, evader_list, object_list, world_radius):
    """ Function to update an evaders state.
        Arguments:
          delta_t (float): The time step size in s.
          evader_list (list of APF_evaders): The autonomous evaders in the world.
          object_list (list of APF_evaders): The objects in the world that evaders should avoid.
          world_radius (float): The radius the world's workspace
    """
    if self.alive:
      apf_force_vector = self.generate_force_vector(evader_list, object_list, world_radius)
      self.actuate_robot(apf_force_vector, delta_t)
    
  def generate_force_vector(self, evader_list, object_list, world_radius):
    """ Function to plot the simulated world.
        Arguments:
          evader_list (list of APF_evaders): The autonomous evaders in the world.
          object_list (list of APF_evaders): The objects in the world that evaders should avoid.
          world_radius (float): The radius the world's workspace
        Returns:
          force_vector (list of 2 floats): The force vector the evader should follow for navigation.
    """
    return self.af3.generate_force_vector(evader_list, object_list, world_radius)

  def actuate_robot(self, apf_force_vector, delta_t):
    """ Convert a force vector to a control signal and actuate the evader like
        a robot.
        Arguments:
          apf_force_vector (list of 2 floats): The 2D force vector.
          delta_t (float): The time step size in s.
    """
    force_angle = math.atan2(apf_force_vector[1], apf_force_vector[0])
    v = self.K_v * math.sqrt(apf_force_vector[0]**2 + apf_force_vector[1]**2 )
    w = self.K_w * angle_diff(force_angle-self.pose.theta)
    v = max(min(v, self.v_max), -self.v_max)
    w = max(min(w, self.w_max), -self.w_max)
    self.pose.x = self.pose.x + v * math.cos(self.pose.theta)*delta_t
    self.pose.y = self.pose.y + v * math.sin(self.pose.theta)*delta_t
    self.pose.theta = self.pose.theta + w*delta_t
  
  def collision(self, evader):
    """ Function to check if the evader is in collision with another evader.
        Arguments:
          evader (APF_evader): The other evader there could be a collision with.
        Returns:
          collision (Bool): True if there is a collision.
    """
    dist = math.sqrt((self.pose.x - evader.pose.x)**2 + (self.pose.y - evader.pose.y)**2)
    return dist < self.radius + evader.radius and self.alive and evader.alive
    
  def remove(self):
    """ Function to reset the pose to be at the origin and set alive to false.
    """
    self.pose = Pose(0, 0, 0)
    self.alive = False
    
  def out_of_bounds(self, world_radius):
    """ Function to check if the evader is out of bounds.
        Arguments:
          world_radius (float): The radius of the world in m.
        Returns:
          out_of_bounds (Bool): True if the evader is out of bounds.
    """
    dist = math.sqrt(self.pose.x**2 + self.pose.y**2)
    return dist > world_radius - self.radius
    
  def at_goal(self):
    """ Function to check if the evader has reached its goal.
        Returns:
          at_goal (Bool): True if the evader is at its goal pose.
    """
    return self.pose.distance_to(self.goal_pose) < goal_threshold

  def update_score(self, time_step):
    """ Function to update an evaders score based on goal reached and collisions.
        Arguments:
          time_step (float): The world's current time step in s.
    """
    if self.alive:
      if self.at_goal():
        self.score = initial_score - time_step
        self.remove()
        print("evader ",self.id,"done with score",self.score)
      elif self.collided:
        self.score = time_step
        self.remove()
        print("evader ",self.id,"done with score",self.score)

class EvaderForce():

    def __init__(self, id):
        self.id = id

    def generate_force_vector(self, evader_list, object_list, world_radius):
  
    # Add attraction to goal
    force = self.attractionForce(evader_list[self.id])
    
    # Add repulsion from other evaders
    for i in range(len(evader_list)):
      if i != self.id:
        force = self.add(force, self.repulsionForce(evader_list[self.id], evader_list[i]))
    
    # Add repulsion from objects
    for i in range(len(object_list)):
      force = self.add(force, self.repulsionForce(evader_list[self.id], object_list[i]))
    
    # Add repulsion from barrier
    d = abs(world_radius - math.sqrt(evader_list[self.id].pose.x **2 + evader_list[self.id].pose.y **2))
    if d < 1:
        x = evader_list[self.id].pose.x
        y = evader_list[self.id].pose.y
        dist = math.sqrt(x **2 + y **2)
        barrier_x = x / dist * world_radius
        barrier_y = y / dist * world_radius
        force = self.add(force, self.repulsionForceToBarrier(evader_list[self.id], [barrier_x, barrier_y]))

    return self.normalize(force)

    def attractionForce(self, evader):
        k_att = 1.5
        # if math.sqrt((evader.goal_pose.x - evader.pose.x)**2 + (evader.goal_pose.y - evader.pose.y)**2) < 3:
        #   k_att = 10
        x_att = k_att * (evader.goal_pose.x - evader.pose.x )
        y_att = k_att * (evader.goal_pose.y - evader.pose.y )
        print("ATT:", [x_att, y_att])
        return [x_att, y_att]
    
    def repulsionForce(self, evader1, evader2):
        k_rep = 200
        rho_0 = 10
        rho_p = math.sqrt((evader1.pose.x - evader2.pose.x)**2 + (evader1.pose.y - evader2.pose.y)**2)
        if (rho_p < rho_0):
        if (rho_p < .5):
            k_rep = k_rep*10
        x_rep = k_rep*((1/(rho_p))-(1/rho_0))*((evader1.pose.x - evader2.pose.x)/(rho_p))
        y_rep = k_rep*((1/(rho_p))-(1/rho_0))*((evader1.pose.y - evader2.pose.y)/(rho_p))
        else:
        x_rep = 0
        y_rep = 0
        print("REP:", [x_rep, y_rep])
        return [x_rep, y_rep]
    
    def repulsionForceToBarrier(self, evader1, closestpt):
            k_rep = 1000
            rho_p = math.sqrt((evader1.pose.x - closestpt[0]) **2 + (evader1.pose.y - closestpt[1]) **2)
            rho_0 = .5
            x_rep = k_rep * ((1 / (rho_p)) - (1 / rho_0)) * ((evader1.pose.x - closestpt[0]) / (rho_p) ** 3)
            y_rep = k_rep * ((1 / (rho_p)) - (1 / rho_0)) * ((evader1.pose.y - closestpt[0]) / (rho_p) ** 3)
            return [x_rep, y_rep]
        
    def add(self, force_1, force_2):
        return [force_1[0] + force_2[0], force_1[1] + force_2[1]]
        
    def normalize(self, vector):
        length = math.sqrt(vector[0]**2 + vector[1]**2)
        return [vector[0] / length, vector[1] / length]

if __name__ == '__main__':
    # for i in range(0, 5):
    # initialize planner
    planner = APFplanner()