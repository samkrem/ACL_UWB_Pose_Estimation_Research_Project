import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from itertools import product
FOLDER_PATH = "~/code/rover_ws/src/rover_trajectory_opt_ros_UWB/rover_trajectory_opt/scripts/"
# FILE_PATH="~/Desktop/ACL_UROP/ACL_UWB_waypoint_generator/snake_sin.csv"
# WAYPOINTS_FILE_PATH = FOLDER_PATH + "circular_sin_waypoints.csv"
# COORDS_FILE_PATH = FOLDER_PATH + "circular_sin_rr_coords.csv"


# WAYPOINTS_FILE_PATH=FOLDER_PATH +"snake_stationary_waypoints.csv"
# COORDS_FILE_PATH="FOLDER_PATH +"snake_stationary_rr_coords.csv"

# WAYPOINTS_FILE_PATH=FOLDER_PATH +"snake_sin_waypoints.csv"
# COORDS_FILE_PATH=FOLDER_PATH +"snake_sin_rr_coords.csv"

# WAYPOINTS_FILE_PATH=FOLDER_PATH +"snake_snake_waypoints.csv"
# COORDS_FILE_PATH=FOLDER_PATH +"snake_snake_rr_coords.csv"

# WAYPOINTS_FILE_PATH=FOLDER_PATH +"circular_stationary_waypoints.csv"
# COORDS_FILE_PATH=FOLDER_PATH +"circular_stationary_rr_coords.csv"
WAYPOINTS_FILE_PATH=FOLDER_PATH +"random_random_rr_coords.csv"
COORDS_FILE_PATH=FOLDER_PATH +"random_random_rr_coords.csv"

WAYPOINTS_DATA_TITLES=["t","x1", "y1","heading1","az1", "el1","x2", "y2", "heading2","az2", "el2",  "dist"]
COORDS_DATA_TITLES = ["t","x1", "y1","heading1","x2", "y2", "heading2"]
def save_waypoints_as_csv(waypoints1, waypoints2):
    """
    Converts waypoints to a CSV file and visualizes the distance histogram.
    Parameters:
    - waypoints1 (numpy.ndarray): 2D array of shape (n, 5) representing (x, y, heading, az, el) coordinates of waypoints for robot 1.
    - waypoints2 (numpy.ndarray): 2D array of shape (m, 5) representing (x, y, heading, az, el) coordinates of waypoints for robot 2.
    - waypoints_titles (list): List of column titles for the CSV file.
    - file_path (str): Path to save the CSV file.
    Returns: None
    """
    # Determine the longer length
    max_length = max(waypoints1.shape[0], waypoints2.shape[0])
    
    # Pad the shorter array by repeating its last row
    def pad_waypoints(waypoints, length):
        if waypoints.shape[0] < length:
            padding = np.tile(waypoints[-1], (length - waypoints.shape[0], 1))
            return np.vstack([waypoints, padding])
        return waypoints
    
    waypoints1 = pad_waypoints(waypoints1, max_length)
    waypoints2 = pad_waypoints(waypoints2, max_length)
    
    # Extract columns
    t = np.arange(max_length)
    x1, y1, heading1, az1, el1 = waypoints1[:, 0], waypoints1[:, 1], waypoints1[:, 2], waypoints1[:, 3], waypoints1[:, 4]
    x2, y2, heading2, az2, el2 = waypoints2[:, 0], waypoints2[:, 1], waypoints2[:, 2], waypoints2[:, 3], waypoints2[:, 4]
    dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    # Create a DataFrame
    df = pd.DataFrame({ WAYPOINTS_DATA_TITLES[0]: t,   
        WAYPOINTS_DATA_TITLES[1]: x1,
        WAYPOINTS_DATA_TITLES[2]: y1,
        WAYPOINTS_DATA_TITLES[3]: heading1,
        WAYPOINTS_DATA_TITLES[4]: az1,
        WAYPOINTS_DATA_TITLES[5]: el1,
        WAYPOINTS_DATA_TITLES[6]: x2,
        WAYPOINTS_DATA_TITLES[7]: y2,
        WAYPOINTS_DATA_TITLES[8]: heading2,
        WAYPOINTS_DATA_TITLES[9]: az2,
        WAYPOINTS_DATA_TITLES[10]: el2,
        WAYPOINTS_DATA_TITLES[11]: dist
    }, columns=WAYPOINTS_DATA_TITLES)  # Include the column titles
    
    # Save the DataFrame to a CSV file
    df.to_csv(WAYPOINTS_FILE_PATH, mode='a', header=False, index=False)
    
    # Plot the distance histogram
    plt.hist(dist, bins=5)
    plt.title("Distance Histogram for Two Robots")
    plt.xlabel("Distance (m)")
    plt.ylabel("Frequency")
    plt.show()

def save_coords_as_csv(coords1, coords2):
    """
    Converts waypoints to csv file
    Parameters:
    - waypoints (numpy.ndarray): 2D array of shape (n, 4) representing (x, y, az, el) coordinates of waypoints.
    Returns: Nothing
    """
    max_length = max(coords1.shape[0], coords2.shape[0])
    
    # Pad the shorter array by repeating its last row
    def pad_coords(coords, length):
        if coords.shape[0] < length:
            padding = np.tile(coords[-1], (length - coords.shape[0], 1))
            return np.vstack([coords, padding])
        return coords
    
    coords1 = pad_coords(coords1, max_length)
    coords2 = pad_coords(coords2, max_length)
    
    t = np.linspace(0,coords1.shape[0]-1, coords1.shape[0])
    x1, y1, heading1 = coords1[:,0], coords1[:,1],coords1[:,2]
    x2, y2, heading2 = coords2[:,0], coords2[:,1],coords2[:,2]

    
    df = pd.DataFrame({
        COORDS_DATA_TITLES[0]: t,   
        COORDS_DATA_TITLES[1]: x1,
        COORDS_DATA_TITLES[2]: y1,
        COORDS_DATA_TITLES[3]: heading1,
        COORDS_DATA_TITLES[4]: x2,
        COORDS_DATA_TITLES[5]: y2,
        COORDS_DATA_TITLES[6]: heading2,
    }, columns=COORDS_DATA_TITLES) # Include the column titles)
    df.to_csv(COORDS_FILE_PATH,  mode='a', header=False, index=False)


def check_intercept(waypoints1, waypoints2):
    """
    Assuming robots travel at same speed, calculates if robots intercept eachother
    """
    x1, y1 = waypoints1[:,0], waypoints1[:,1]
    x2, y2 = waypoints2[:,0], waypoints2[:,1]
    for i in range(len(x1)-1):
        crash = (x1[i+1], y1[i+1]) == (x2[i], y2[i]) or (x1[i], y1[i]) == (x2[i], y2[i]) #or (x1[i], y1[i]) == (x2[i+1], y2[i+1])

        if crash:
            if (x1[i+1], y1[i+1]) == (x2[i], y2[i]):
                print(f"Robots hit eachother between step {i}-{i+1} around {(x1[i], y1[i])}-{(x1[i+1], y1[i+1])}")
            else:
                print(f"Robots hit eachother between step {i} at {(x1[i], y1[i])}")
            return True
    return False

def plot_robots_trajectory_with_direction(waypoints1, waypoints2):
    """
    Plots the trajectory of two robots with arrows indicating its direction at each waypoint.

    Parameters:
    - waypoints (numpy.ndarray): 2D array of shape (n, 4) representing (x, y, az, el) coordinates of waypoints.
    Returns: Nothing
    """

    plot_robot_trajectory(waypoints1, "Robot 1")
    plot_robot_trajectory(waypoints2, "Robot 2")
    ax = plot_robot_trajectory(waypoints1, "Robot 1")
    plot_robot_trajectory(waypoints2, "Robot 2", ax)
    plt.show()
def plot_robot_trajectory(waypoints, robot_name, ax = None):
    """
    Plot trajectory of individualrobot
    """
    if ax == None:
        fig, ax = plt.subplots(figsize=(6, 6))
        plot_title = f'{robot_name} Trajectory'
    else:
        plot_title = 'Robots 1 and 2 Trajectory'
    if robot_name == "Robot 1":
        line_color = "blue"
        vector_color = "blue"
    if robot_name == "Robot 2":
        line_color = "purple"
        vector_color = "purple"
    ax.plot(waypoints[:, 0], waypoints[:, 1], c= line_color ,label=f'{robot_name} Path', linestyle='-')     # Plot the trajectory using lines
    ax.scatter(waypoints[:, 0], waypoints[:, 1], c= vector_color, label=f'{robot_name} Waypoints', marker='o')    # Plot individual dots at each waypoint
    ax.scatter(waypoints[0, 0], waypoints[0, 1], c='green', label=f'{robot_name} Start', marker='o', s=100) # Green for start
    ax.scatter(waypoints[-1, 0], waypoints[-1, 1], c='red', label=f'{robot_name} End', marker='o', s=100) # Red for end
    
    # Calculate arrow components based on differences between consecutive waypoints
    dx = np.diff(waypoints[:,0])
    dy = np.diff(waypoints[:,1])
    
    # Plot arrows for the direction at each waypoint using plt.quiver
    plt.quiver(waypoints[:-1, 0], waypoints[:-1, 1],dx, dy, color= vector_color, scale=2, scale_units='xy', angles='xy', headwidth=10, headlength=10)
    
    # Customize the plot (add labels, title, legend, etc.)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(plot_title)
    ax.legend(loc='upper left', bbox_to_anchor=(-0.1, 1.15))
    ax.grid(True)
    return ax
    
def define_waypoint_order_grid(x_points, y_points, option):
    """
    Defines order of x,y point  
    
    Parameters:
    - x_points (numpy.ndarray): range of x coordinates with step size
    - y_points (numpy.ndarray): range of y coordinates with step size
    - option: way turtlebot moves for exhaustive version
    Returns: Final Coordinate
    """
    x_points_new, y_points_new = None, None
    #pick option
    if option in ("snake_1", "snake_5"): 
        x_points_new, y_points_new = x_points, y_points
    elif option in ("snake_2", "snake_6"): 
        x_points_new, y_points_new = x_points[::-1], y_points
    elif option in ("snake_3", "snake_7"): 
        x_points_new, y_points_new = x_points[::-1], y_points[::-1]
    elif option in ("snake_4", "snake_8"):
        x_points_new, y_points_new = x_points, y_points[::-1]
    if option in ("snake_1", "snake_2", "snake_3", "snake_4"):
        increment = "lr_movement" # x coord increases and y coord is constant 
    else: 
        increment = "ud_movement" # y coord increases and x coord is constant 
    #optimize based on option
    coordinates_final = optimize_waypoint_order_grid(increment, x_points_new,y_points_new)
    return coordinates_final
def optimize_waypoint_order_grid(increment, x_points, y_points):
    """
    Optimizes order of x,y point order such
    
    Parameters:
    - x_points (numpy.ndarray): range of x coordinates with step size
    - y_points (numpy.ndarray): range of y coordinates with step size
    - increment (string): specifices which direction turtlebot initially moves

    Returns: optimized waypoints
    """
    if increment == "lr_movement":
        coordinates = np.array([(x, y) for y in y_points for x in x_points])
    else:
        coordinates = np.array([(x, y) for y in y_points for x in x_points])
    coordinates= coordinates.reshape((len(y_points),len(x_points),2)) 

    length = coordinates.shape[0] if increment == "lr_movement" else coordinates.shape[1]
    final_ordered_coords = []
    for i in range(length):     
        line = coordinates[i, :,:] if increment == "lr_movement" else coordinates[:,i,:]#row of our np array 
        if i%2!=0:
             line = line[::-1]              
        final_ordered_coords.append(line)
    return np.concatenate(final_ordered_coords, axis=0) #after rearranging order of coord shape into sequential order ie [(x1,y1), (x2,y2)]

def generate_exhaustive_waypoints(params, bounds):
    """
    Generates exhaustive waypoints (ie every permutation of (x,y,el,az))
    Parameters:
        - params (tuple): (start, option_choice)
        - bounds (tuple): tuple of x,y,yaw,azimuth, and elevation dictionarys which describe the robots physical bounds
    Returns: waypoints
    xy_params = 
    """
    start, option_choice = params
    default_bounds = {'length': 8, 'step': 81} , {'length': 8, 'step': 81}, {'min': -180, 'max': 180, 'step': 5}, {'min': -180, 'max': 180, 'step': 5}, {'min': -120, 'max': 120, 'step': 5}
    x_spec, y_spec, az_spec, el_spec = bounds or default_bounds #yaw_spec
   
    x_points, y_points = np.linspace(0,x_spec['length'], x_spec["step"]), np.linspace(0,y_spec['length'], y_spec["step"])
    coordinates = start + define_waypoint_order_grid(x_points, y_points, option_choice)
     
    waypoints, rr_waypoints, metronome_poses_per_coord = create_trans_rot_waypoint(coordinates, az_spec, el_spec) #yaw_spec
    return waypoints, rr_waypoints, metronome_poses_per_coord
def generate_stationary_waypoints(params,bounds):
    """
    Generates stationary waypoints based on specified parameters.

    Parameters:
    - params (list): List containing the coordinate and total_xy_points.
    - bounds (tuple): tuple of azimuth, and elevation dictionarys which describe the robots physical bounds
    

    Returns:
    - waypoints (array): Array of waypoints combining coordinates and orientation.
    """
    stationary_point, total_xy_points = params[0], params[1]
    default_bounds =  {'min': 40, 'max': 40, 'step': 1}, {'min': 0, 'max': 0, 'step': 1} #{'min': 0, 'max': 0, 'step': 1},
    az_spec, el_spec = bounds or default_bounds #, yaw_spec
   
    coordinates = np.tile(stationary_point, (total_xy_points,1))
    waypoints, rr_waypoints, metronome_poses_per_coord = create_trans_rot_waypoint(coordinates, el_spec, az_spec) #, yaw_spec
    return waypoints, rr_waypoints, metronome_poses_per_coord

def generate_circular_waypoints(params, bounds):
    """
    Generates circular waypoints based on specified parameters.

    Parameters:
    - params (list): List containing the start point, total_xy_points, dr, and num_circles.
    - bounds (tuple): tuple of yaw,azimuth, and elevation dictionarys which describe the robots physical bounds

    Returns:
    - waypoints (array): Array of waypoints combining coordinates and orientation.
    """
    start, total_xy_points, dr, num_circles = params
    default_bounds =  {'min': -180, 'max': 180, 'step': 5}, {'min': -120, 'max': 120, 'step': 5} #{'min': -180, 'max': 180, 'step': 5},
    az_spec, el_spec = bounds or default_bounds #yaw_spec,
    
    coordinates = []
    circumference_sum = 2 * np.pi * sum([(i + 1) * dr for i in range(num_circles)])

    # for i in range(num_circles):
    #     r = (i + 1) * dr
    #     circle_circum = 2 * np.pi * r
    #     points_per_circle = int(total_xy_points * circle_circum / circumference_sum)
    #     angles = np.linspace(0, 2 * np.pi, points_per_circle)
    #     x, y = r * np.cos(angles), r * np.sin(angles)
    #     indiv_circle = np.column_stack((x, y))
    #     coordinates.extend(indiv_circle)
    points_per_circle = total_xy_points // num_circles
    for i in range(num_circles):
        r = (i + 1) * dr
        # circle_circum = 2 * np.pi * r
        # points_per_circle = int(total_xy_points * circle_circum / circumference_sum)
        angles = np.linspace(0, 2 * np.pi, points_per_circle)
        x, y = r * np.cos(angles), r * np.sin(angles)
        indiv_circle = np.column_stack((x, y))
        coordinates.extend(indiv_circle)


    left_over_count = total_xy_points - len(coordinates)
    last_points = np.tile(coordinates[-1], (left_over_count, 1))
    coordinates.extend(last_points)
    coordinates = start + np.array(coordinates)

    waypoints, rr_waypoints, metronome_poses_per_coord = create_trans_rot_waypoint(coordinates, el_spec, az_spec, points_per_circle = points_per_circle)
    return waypoints, rr_waypoints, metronome_poses_per_coord
def generate_sinusoidal_waypoints(params, bounds = None):
    """
    Generates sinusoidal waypoints based on specified parameters.

    Parameters:
    - params (list): List containing the start point, total_xy_points, num_waves, dist, and option.
    - bounds (tuple): tuple of x,y ,azimuth, and elevation dictionarys which describe the robots physical bounds
    Returns:
    - waypoints (array): Array of waypoints combining coordinates and orientation.
    """
    start, total_xy_points, num_waves, freq, amp, dist, option = params
    print(params)
    default_bounds = {'length': 8, 'step': 81}, {'length': 8, 'step': 81},  {'min': -180, 'max': 180, 'step': 5}, {'min': -80, 'max': 80, 'step': 5} #{'min': -180, 'max': 180, 'step': 5},
    x_spec, y_spec,  az_spec, el_spec = bounds or default_bounds 
    # print(az_spec, el_spec)
    coordinates = []
    points_per_wave = total_xy_points//num_waves

    for i in range(num_waves):
        if option == "horizontal":
            x = np.linspace(0,x_spec['length'],points_per_wave)
            y = amp* np.sin(freq*2*np.pi*x)+dist*i 
        else:
            y = np.linspace(0,y_spec['length'],points_per_wave)
            print("y:",y, len(y))
            x = amp*np.sin(freq*2*np.pi*y)+ dist*i       
        indiv_wave_coord = np.column_stack((x,y))
        if i%2 != 0:
            indiv_wave_coord = np.flipud(indiv_wave_coord)
    
        coordinates.extend(indiv_wave_coord)
    left_over_count = total_xy_points % num_waves #so we reach total points
    last_points = np.tile(coordinates[-1], (left_over_count,1))
    coordinates.extend(last_points)

    coordinates = start + np.array(coordinates)
    waypoints, rr_waypoints, metronome_poses_per_coord = create_trans_rot_waypoint(coordinates, az_spec, el_spec)
    return waypoints, rr_waypoints, metronome_poses_per_coord
def generate_randomized_waypoints(params, bounds = None):
    default_bounds = {'min': -8, 'max' :8,'step': 81}, {'min': -8, 'max' :8,'step': 81}, {'min': -180, 'max': 180, 'step': 5}, {'min': -120, 'max': 120, 'step': 5}
    x_spec, y_spec, az_spec, el_spec = bounds or default_bounds

    num_rows = x_spec['step']*y_spec['step']
    rng = np.random.default_rng()
    x_coords = rng.uniform(low=x_spec['min'], high=x_spec['max'], size=num_rows)
    y_coords = rng.uniform(low=y_spec['min'], high=y_spec['max'], size=num_rows)

    # Create coordinates array
    coordinates = np.column_stack((x_coords, y_coords))
    waypoints, rr_waypoints, metronome_poses_per_coord = create_trans_rot_waypoint(coordinates, az_spec, el_spec)
    
    return waypoints, rr_waypoints, metronome_poses_per_coord
def create_circuluar_traj_headings(coordinates, points_per_circle):
    dxs = np.diff(coordinates[:, 0])
    dys = np.diff(coordinates[:, 1])

    dxs = np.append(dxs, 0)
    dys = np.append(dys, 0)
    
    # Calculate initial heading
    angle = np.arctan2(dys[0], dxs[0])
    prev_heading = angle if angle >= 0 else angle + 2 * np.pi
    headings = [prev_heading]
    
    print(points_per_circle)
    
    for i in range(1, len(coordinates)):
        if dxs[i] == 0 and dys[i] == 0:
            headings.append(headings[-1])
        else:
            angle = np.arctan2(dys[i],dxs[i]) 
            heading = angle if angle >=0 else angle + 2 *np.pi

            # circle_number =  i // points_per_circle
            # heading += 2 * np.pi * circle_number
            diff = heading - prev_heading
            while diff > np.pi or diff < -np.pi:
                if diff > np.pi:
                    heading -= 2 * np.pi  # Take the shorter path by subtracting 2pi
                elif diff < -np.pi:
                    heading += 2 * np.pi  # Take the shorter path by adding 2pi
                diff = heading - prev_heading  # Recalculate the difference after adjustment
            headings.append(heading)

            prev_heading = heading
    
    return np.array(headings)



def create_headings(coordinates
                    ):
    dxs = np.diff(coordinates[:,0])
    dys = np.diff(coordinates[:,1])

    dxs = np.append(dxs, 0)
    dys = np.append(dys, 0)
    prev_dx, prev_dy = dxs[0], dys[0]

    # headings = [np.arctan2(prev_dy, prev_dx)] #if np.arctan2(prev_dy, prev_dx) >= 0 else [2*np.pi + np.arctan2(prev_dy, prev_dx)]
    angle = np.arctan2(dys[0],dxs[0])
    prev_heading = angle if angle >=0 else angle + 2 *np.pi
    headings =[prev_heading]
    for i in range(1,len(coordinates)):
        if dxs[i]== 0 and dys[i] == 0:
            headings.append(headings[-1])
        
        else:
            angle = np.arctan2(dys[i],dxs[i])
            heading = angle if angle >=0 else angle + 2 *np.pi
            diff = heading - prev_heading
            if diff > np.pi:
                heading -= 2 * np.pi  # Wrap around the heading
            elif diff < -np.pi:
                heading += 2 * np.pi
            headings.append(heading)
            prev_heading = heading

    return headings
def create_trans_rot_waypoint(coordinates, az_spec, el_spec, points_per_circle = 0):
    """
    Combine translational (x,y) waypoints with rotational permutations at those waypoints.

    Parameters:
    - coordinates (array): Array of (x, y) translational waypoints.
    - bounds (tuple): tuple of yaw,azimuth, and elevation dictionarys which describe the robots physical bounds

    Returns:
    - waypoints (array): Array of waypoints combining coordinates and orientation.
    """
    az_points = np.linspace(az_spec["min"],az_spec["max"],az_spec["step"])
    el_points = np.linspace(el_spec["min"], el_spec["max"], el_spec["step"])
    # print(az_points)
    # print(el_points)
    if points_per_circle == 0:
        headings = np.array(create_headings(np.array(coordinates)))
    else:
        headings = np.array(create_circuluar_traj_headings(np.array(coordinates), points_per_circle))
    # waypoints = np.insert(waypoints, 2, headings, axis=1)
    coord_az_el_permutation=list(product(zip(coordinates, headings), az_points, el_points))
    # print(len(coordinates), len(headings), len(_)
    waypoints = np.array([
        np.hstack((coord,heading, az, el))
        for (coord,heading), az, el in coord_az_el_permutation
    ])

    rr_waypoints = np.column_stack((coordinates, headings))
    metronome_poses_per_coord = (list(product(az_points,el_points)))

    return waypoints, rr_waypoints, metronome_poses_per_coord

def custom_paths(option_list, bounds_list, params_list):
    """
    Generate custom path for two robots
    """
    
    waypoints_list = []
    for option, bounds, params in zip(option_list,bounds_list, params_list):
        if option == "Stationary":
            waypoints_list.append(generate_stationary_waypoints(params, bounds))
        elif option == "Exhaustive":
            waypoints_list.append(generate_exhaustive_waypoints(params, bounds))
        elif option == "Sinusoidal":
            waypoints_list.append(generate_sinusoidal_waypoints(params, bounds))
        elif option == "Circular":
            waypoints_list.append(generate_circular_waypoints(params, bounds))
        else:
            waypoints_list.append(generate_randomized_waypoints(params,bounds))
    return waypoints_list
#take in two options based on those options spit out params
def generate_params_bounds(option1, option2):
    """
    Generate robot parameters and bounds 
    """

    if option1 == "Stationary":
        params1 = ((0,0), 600)
        bounds1 =  {'min': 0, 'max': 0, 'step': 1}, {'min': 0, 'max': 0, 'step': 1} #{'min': 0, 'max': 0, 'step': 1},
    elif option1 == "Exhaustive":
        params1 = ((0,0), "snake_1")
        bounds1 = {'length': 8, 'step': 16}, {'length': 8, 'step': 16},  {'min': -150, 'max': 150, 'step': 5}, {'min': -80, 'max': 80, 'step': 5}  #{'min': -180, 'max': 180, 'step': 1},
    elif option1 == "Sinusoidal":
        sin_start1, num_points1, num_waves1, freq1, amp1, wave_dist1, direction1 =(-7, -11), 25, 3,.2,3, 6, "vertical"
        params1 = (sin_start1, num_points1 ,num_waves1, freq1, amp1, wave_dist1, direction1)
        bounds1 = {'length': 8, 'step': 15}, {'length': 8, 'step': 15}, {'min': -150, 'max': 150, 'step': 10}, {'min': -80, 'max': 80, 'step': 10}  # {'min': -180, 'max': 180, 'step': 1},
    elif option1 == "Circular":
        circle_start1, num_points1, dr1, num_circles1 = (0,0), 100, 1, 4
        params1 = (circle_start1, num_points1, dr1, num_circles1)
        bounds1 =  {'min': -150, 'max': 150, 'step': 10}, {'min': -80, 'max': 80, 'step': 10} #{'min': -180, 'max': 180, 'step': 1},
    else:
        params1 = ()
        bounds1 = {'min': -8, 'max': 8, 'step': 10}, {'min': -8, 'max': 8, 'step': 10}, {'min': -180, 'max': 180, 'step': 5}, {'min': -120, 'max': 120, 'step': 5}
    if option2 == "Stationary":
        params2 = ((0,0), 1)
        bounds2 =  {'min': -150, 'max': 150, 'step': 5}, {'min': -80, 'max': 80, 'step': 5} #{'min': 0, 'max': 180, 'step': 1},
    elif option2 == "Exhaustive":
        params2 = ((0,0), "snake_8")
        bounds2 = {'length': 8, 'step': 16}, {'length': 8, 'step': 16}, {'min': -150, 'max': 150, 'step': 3}, {'min': -80, 'max': 80, 'step': 5} # {'min': -180, 'max': 180, 'step': 1},
    elif option2 == "Sinusoidal":
        sin_start2, num_points2, num_waves2, freq2, amp2, wave_dist2, direction2 =(-7, -11), 64, 3,.2,3, 6, "vertical"
        params2 = (sin_start2, num_points2 ,num_waves2, freq2, amp2,wave_dist2, direction2)
        bounds2 = {'length': 8}, {'length': 8}, {'min': -150, 'max': 150, 'step': 5}, {'min': -80, 'max': 80, 'step': 5}  #{'min': -180, 'max': 180, 'step': 1}, 
    elif option2 == "Circular":
        circle_start2, num_points2, dr2, num_circles2 = (0,0),300, 5, 3
        params2 = (circle_start2, num_points2, dr2, num_circles2)
        bounds2 =  {'min': 0, 'max': 180, 'step': 10}, {'min': 0, 'max': 180, 'step': 10} #{'min': 0, 'max': 180, 'step': 3},
    else:
        params2 = ()
        bounds2 = {'min': -8, 'max': 8, 'step': 10}, {'min': -8, 'max': 8, 'step': 10}, {'min': -180, 'max': 180, 'step': 5}, {'min': -120, 'max': 120, 'step': 5}

    return params1, params2, bounds1, bounds2


def generate_waypoints():
    # df_waypoints = pd.DataFrame(columns=WAYPOINTS_DATA_TITLES)  # Include the column titles)
    # df_waypoints.to_csv(WAYPOINTS_FILE_PATH,  mode='a', header=True, index=False)
    
    df_coords = pd.DataFrame(columns=COORDS_DATA_TITLES)  # Include the column titles)
    df_coords.to_csv(COORDS_FILE_PATH,  mode='a', header=True, index=False)
    
    #Example Params
    path_options = ["Stationary", "Exhaustive", "Sinusoidal", "Circular", "Randomized"]
    option1, option2 = path_options[4], path_options[4]

    option1_params, option2_params, bounds1, bounds2, = generate_params_bounds(option1, option2)

   
    # option3, option4 = path_options[2], path_options[3]
    # option3_params, option4_params, bounds3, bounds4, = generate_params_bounds(option3, option4)
   
    # #To concatenate paths together use (np.concatenate(waypoints_list, axis=0) )

    r1_data, r2_data = custom_paths((option1, option2), (bounds1, bounds2), (option1_params, option2_params))
    r1_waypoints, r1_rr_coords, r1_metronome_poses_per_coord = r1_data
    r2_waypoints, r2_rr_coords, r2_metronome_poses_per_coord = r2_data
    # morewaypoints1, morewaypoints2 = custom_paths((option3, option4), (bounds3, bounds4), (option3_params, option4_params))
    # r1_waypoints =   np.concatenate([r1_waypoints, morewaypoints1], axis=0)
    # r2_waypoints =   np.concatenate([r2_waypoints, morewaypoints2], axis=0)
    # save_waypoints_as_csv(r1_waypoints, r2_waypoints)

    save_coords_as_csv(r1_rr_coords, r2_rr_coords)

    plot_robots_trajectory_with_direction(r1_waypoints, r2_waypoints)

    
    #Can also manually generate waypoints without use of functions
    # r1_waypoints = generate_exhaustive_waypoints(exhaustive_params1, x_spec=x_spec, y_spec=y_spec, yaw_spec=yaw_spec, az_spec=az_spec, el_spec=el_spec)
    # r2_waypoints = generate_exhaustive_waypoints(exhaustive_params2, x_spec=x_spec2, y_spec=y_spec2, yaw_spec=yaw_spec, az_spec=az_spec, el_spec=el_spec)

    # r2_waypoints = generate_stationary_waypoints(stationary_params, yaw_spec=yaw_spec, az_spec=az_spec, el_spec=el_spec)
    # r1_waypoints = generate_sinusoidal_waypoints(sinusoidal_params, x_spec=x_spec, y_spec=y_spec, yaw_spec=yaw_spec, az_spec=az_spec, el_spec=el_spec)
    # r2_waypoints = generate_circular_waypoints(circular_params, yaw_spec=yaw_spec, az_spec=az_spec,  el_spec=el_spec)
    #to make r1 or r2 waypoints multiple paths can do result = np.concatenate(arrays_list, axis=0)
    # sinusoidal_waypoints = generate_sinusoidal_waypoints(sinusoidal_params, x_spec=x_spec, y_spec=y_spec, yaw_spec=yaw_spec, az_spec=az_spec, el_spec=el_spec)
    # last_sinusoidal_point = sinusoidal_waypoints[-1,:2]
    # circular_params = (last_sinusoidal_point, 500, dr, num_circles)
    # circular_waypoints = generate_circular_waypoints(circular_params, yaw_spec=yaw_spec, az_spec=az_spec, el_spec=el_spec)
    #r1_waypoints = np.concatenate(waypoints_list, axis=0)
    #r2_waypoints = generate_stationary_waypoints(stationary_params, yaw_spec=yaw_spec, az_spec=az_spec, el_spec=el_spec)

if __name__ == "__main__":
    generate_waypoints()



