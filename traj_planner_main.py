# E206 Motion Planning

# Combined trajectory planner (evader and chaser)
# Daphne Poon and Sabrina Shen

from traj_planner_APF import APF_Planner
from traj_planner_MTPP import Moving_Target_Path_Planner


class Combined_Planner():

    def __init__(self):
        self.evader_planners = []
        self.chaser_planner = Moving_Target_Path_Planner()
        pass

    def initialize_evaders(self, num):
        for _ in range(num):
            self.evader_planners.append(APF_Planner())
        pass


if __name__ == '__main__':
    pass
