import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from tomma.multi_agent_optimization import MultiAgentOptimization
from tomma.dubins_dynamics import DubinsDynamics, CONTROL_LIN_ACC_ANG_VEL


from create_and_store_trajectories import create_and_store_trajectories

# parameters
num_rovers = 2
num_timesteps = 18 #11 for snake sin
min_allowable_dist = .2

XMIN = -10
XMAX = 10
YMIN = -10
YMAX = 10

XMID = (XMAX + XMIN) / 2
YMID = (YMAX + YMIN) / 2


x_bounds = np.array([[XMIN, XMAX], [YMIN, YMAX], [-.25, 1], [-np.inf ,np.inf]])
u_bounds = np.array([[-.25, .25], [-.785, .785]])

FILE_NAME = "random_random_rr_coords.csv"

df = pd.read_csv(FILE_NAME, header = 0)
waypoints = np.array([
    df.iloc[:,1:4],df.iloc[:,4:7]
])

output_file = '/home/swarm/data/trajectories/random_random.json'

# setup

planner = MultiAgentOptimization(dynamics=DubinsDynamics(control=CONTROL_LIN_ACC_ANG_VEL), 
                                num_agents=num_rovers, 
                                num_timesteps=num_timesteps,
                                min_allowable_dist=min_allowable_dist,
                                x_bounds=x_bounds,
                                u_bounds=u_bounds)


create_and_store_trajectories(planner=planner, num_rovers=num_rovers, waypoints=waypoints, output_file=output_file)
plt.show()
