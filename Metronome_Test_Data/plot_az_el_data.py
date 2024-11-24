from matplotlib import pyplot as plt
import pandas as pd
import numpy as np
from itertools import product
import argparse

def plot_csv_data(csv_file, title, met_id=1, plot3d=False):
    data = pd.read_csv(csv_file)
    az_vals = data["yaw"]
    elev_vals = data["pitch"]
    roll_vals = data["roll"]
    
    # fig = plt.figure(1)
    yticks = np.linspace(-180, 180, 13)
    
    ## Operational Range Points
    az_el_vals_permutations = list(product(np.linspace(-150, 150, 31), np.linspace(-80, 80, 17))) # change linspace start and end to correspond with min and max of elev range
    az_el_arr = np.array([np.hstack(az_el) for az_el in az_el_vals_permutations])
    az_roll_vals_permutations = list(product(np.linspace(-150, 150, 31), np.zeros((1, 1)))) # change linspace start and end to correspond with min and max of elev range
    az_roll_arr = np.array([np.hstack(az_roll) for az_roll in az_roll_vals_permutations])
    roll_elev_vals_permutations = list(product(np.zeros((1, 1)), np.linspace(-80, 80, 17) )) # change linspace start and end to correspond with min and max of elev range
    roll_elev_arr = np.array([np.hstack(roll_elev) for roll_elev in roll_elev_vals_permutations])

    
    ## Target Points
    num_pts = 6 # default for met 1
    if met_id == 2:
        num_pts = 8
    target_az = np.linspace(-150, 150, num_pts)
    target_el = np.linspace(-80, 80, num_pts)
    target_roll = np.zeros((1,1))
    target_az_el_permutations = list(product(target_az, target_el))
    target_az_el_arr = np.array([np.hstack(az_el) for az_el in target_az_el_permutations])
    target_az_roll_permutations = list(product(target_az, target_roll))
    target_az_roll_arr = np.array([np.hstack(az_roll) for az_roll in target_az_roll_permutations])
    target_roll_elev_permutations = list(product(target_roll, target_el))
    target_roll_elev_arr = np.array([np.hstack(roll_elev) for roll_elev in target_roll_elev_permutations])

    
    if plot3d:
        fig = plt.figure(1)
        ax = fig.add_subplot(projection='3d')
        
        ## Operational Range points
        ax.scatter(az_el_arr[:,0], az_el_arr[:,1], np.zeros((az_el_arr.shape[0], 1)), c='green', label='Operational Range')
        
        ## Plot target points
        ax.scatter(target_az_el_arr[:, 0], target_az_el_arr[:, 1], np.zeros((target_az_el_arr.shape[0], 1)), c = 'magenta', s= 100, label='Target')
        
        ## Plot experimental data
        ax.scatter(az_vals, elev_vals, roll_vals, c = 'blue', label='Experimental')
        ax.scatter(az_vals[0], elev_vals[0], roll_vals[0], c='red', marker='<', s=100, label='start')
        ax.scatter(az_vals.iloc[-1], elev_vals.iloc[-1], roll_vals.iloc[-1], c='orange', label='end')
        
        ax.set_xlabel('Azimuth [deg]')
        ax.set_ylabel('Elevation [deg]')
        ax.set_zlabel('Roll [deg]')
        
        ax.set_title(f'{title[:]}: Elevation vs Azimuth w/ Roll')
        plt.xticks(ticks=np.linspace(-180, 180, 13))
        plt.yticks(ticks=np.linspace(-90, 90, 13))
         
    else:
        fig, ax = plt.subplots(2,2)
        # ax = plt.axes
        fig.set_figheight(10)
        fig.set_figwidth(16)
        
        ## Operational Range points
        ax[0,0].scatter(az_el_arr[:,0], az_el_arr[:,1], c='green', label='Operational Range')
        ax[1,0].scatter(az_roll_arr[:,0], az_roll_arr[:, 1], c = 'green',  label='Operational Range' )
        ax[0, 1].scatter(roll_elev_arr[:, 0], roll_elev_arr[:, 1],  c = 'green',  label='Operational Range' )
 
        
        ## Plot target points
        ax[0, 0].scatter(target_az_el_arr[:, 0], target_az_el_arr[:, 1], c = 'magenta', s= 100, label='Target')
        ax[1, 0].scatter(target_az_roll_arr[:, 0], target_az_roll_arr[:, 1], c = 'magenta', s= 100, label='Target')
        ax[0, 1].scatter(target_roll_elev_arr[:, 0], target_roll_elev_arr[:, 1], c = 'magenta', s= 100, label='Target')

        
        ## Plot experimental data
        ax[0, 0].scatter(az_vals, elev_vals, c = 'blue', label='Experimental')
        ax[0, 0].scatter(az_vals[0], elev_vals[0], c='red', marker='<', s=100, label='start')
        ax[0, 0].scatter(az_vals.iloc[-1], elev_vals.iloc[-1], c='orange', label='end')
        
        ax[1, 0].scatter(az_vals, roll_vals, c = 'blue', label='Experimental')
        ax[1, 0].scatter(az_vals[0], roll_vals[0], c='red', marker='<', s=100, label='start')
        ax[1, 0].scatter(az_vals.iloc[-1], roll_vals.iloc[-1], c='orange', label='end')
        
        ax[0, 1].scatter(roll_vals, elev_vals, c = 'blue', label='Experimental')
        ax[0, 1].scatter(roll_vals[0], elev_vals[0], c='red', marker='<', s=100, label='start')
        ax[0, 1].scatter(roll_vals.iloc[-1], elev_vals.iloc[-1], c='orange', label='end')
        
        ax[0, 0].set_xlabel('Azimuth [deg]')
        ax[1, 0].set_xlabel('Azimuth [deg]')
        ax[0, 1].set_xlabel('Roll [deg]')
        
        
        ax[0, 0].set_ylabel('Elevation [deg]')
        ax[1, 0].set_ylabel('Roll [deg]')
        ax[0, 1].set_ylabel('Elevation [deg]')
        
        # ax[0, 0].set_title(f'{title[:]}: Elevation vs Azimuth')
        # ax[1, 0].set_title(f'{title[:]}: Roll vs Azimuth')
        # ax[0, 1].set_title(f'{title[:]}: Roll vs Elevation')
        
        ax[0, 0].grid(True)
        ax[1, 0].grid(True)
        ax[0, 1].grid(True)
        
        # ax[0, 0].legend()
        # ax[1, 0].legend()
        ax[0, 1].legend()
        
        fig.suptitle(f'{title}: Roll, Pitch, Yaw')
        
        ax[0,0].set_yticks(ticks=np.linspace(-90, 90, 19))
        ax[0,1].set_yticks(ticks=np.linspace(-90, 90, 19))
        
        
    # plt.xticks(ticks=np.linspace(-180, 180, 13))
    # plt.yticks(ticks=np.linspace(-90, 90, 13))
    # ax.grid(True)
    # ax.legend()
    plt.savefig(title + '.png', format='png')


# filename = 'met2v2_seg3.csv'
# plot_csv_data(filename, 'Metronome 2 v2 Trial Run 3')

if __name__ == '__main__':
    print("Plot metronome data")
    ## 1) Plot data from csv files
    parser = argparse.ArgumentParser()
    parser.add_argument('csv_files',nargs='*')
    args = parser.parse_args()
    csv_files = args.csv_files
    
    ## Running py file in VsCode, no command-line args --> supply bag file names here
    if len(csv_files) == 0:
        csv_files = []
    
    for file in csv_files:
        file_str = str(file)
        file_str = file_str.strip('metv3/ishberg_metv3_trial')
        file_str = file_str.strip('-world-ZYX.csv')
        if file_str == '':
            file_str = '3'
        plot_title = 'Met1 v3 Trial ' + file_str
        plot_csv_data(file, title=plot_title)