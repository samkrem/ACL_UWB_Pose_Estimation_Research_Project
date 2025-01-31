import numpy as np
import json
import matplotlib.pyplot as plt

def create_and_store_trajectories(planner, num_rovers, waypoints, output_file):
    """
    Creates and stores a sequence of trajectories as a json file

    Args:
        planner (MultiAgentOptimzation): tomma MultiAgentOptimization planner.
        num_rovers (int): number of rovers that need trajectories
        waypoints (array-like, shape=(num_rovers, n, 3)): n 2D + heading waypoints for each rover 
            to travel to
        # objects (array-like, shape=(m, 3)): m 2D object locations (indices 0,1) and 
        #     radius (index 2)
        output_file (str): output file path
    """
    
    assert num_rovers == len(waypoints), "waypoints shape is incorrect"
    assert planner.M == len(waypoints), "waypoints shape is incorrect"

    fig, ax = plt.subplots()

    num_paths = waypoints.shape[1] - 1
    
    # num_paths x num_agents x num_timesteps+1 x (len(x) + len(u))
    trajectory = np.zeros((num_paths, num_rovers, planner.N+1, planner.dynamics.x_shape + planner.dynamics.u_shape))*np.nan
    # num_paths x num_timesteps+1
    times = np.zeros((num_paths, planner.N+1))
    
    for i in range(num_paths):
        # zero velocity at start and end
        x0 = np.hstack([waypoints[:,i,:2], np.zeros((num_rovers, 1)), waypoints[:,i,2:]])
        xf = np.hstack([waypoints[:,i+1,:2], np.zeros((num_rovers, 1)), waypoints[:,i+1,2:]])

        # print(x0)
        # for robot_i in range(waypoints.shape[0]):
        #     yaw0_i = x0[robot_i,3]
        #     yawf_i = xf[robot_i,3]
        #     print(yaw0_i)
        #     print(yawf_i)
        #     while yaw0_i - yawf_i < -np.pi:
        #         yawf_i -= 2*np.pi
        #     while yawf_i - yaw0_i < -np.pi:
        #         yawf_i += 2*np.pi
        #     xf[robot_i,3] = yawf_i
        #     print(yawf_i)
        # if i == 7:
        #     import ipdb; ipdb.set_trace()
        #     x0[0,1] = 4.1

        # solve
        planner.setup_min_time_opt(x0, xf, tf_guess=10.)
        planner.opti.subject_to(planner.tf > 1.)
        x, u, t = planner.solve_opt()

        print('cool')
        fig, ax = planner.draw_path(fig, ax)
        for j in range(num_rovers):
            # Extract the (x, y) coordinates for the current rover's waypoints
            rover_waypoints = waypoints[j, :, :2]  # Only take x, y for plotting (excluding heading)
            ax.scatter(rover_waypoints[:, 0], rover_waypoints[:, 1], label=f'Rover {j+1} Waypoints', s=3, marker='o', zorder=5)

        # store
        times[i,:] = t
        for j in range(num_rovers):
            trajectory[i,j,:,:4] = np.array(x[j]).T
            trajectory[i,j,:-1,4:] = np.array(u[j]).T # one less control command than state
            
    with open(output_file, 'w') as f:
        json.dump({'trajectory': trajectory.tolist(), 'time': times.tolist()}, f)

    