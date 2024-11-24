import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np
import time
import datetime
import argparse

# importing sys
# import sys

# adding Folder_2/subfolder to the system path
# sys.path.insert(0, 'C:\Users\Julianne Miana\Desktop\Fishberg UROP Su24\uropsu24\Metronome_Test_Data\fishberg_metv2')
# from plot_az_el_data import *

from scipy.spatial.transform import Rotation as Rot

DATA_TITLES = ["x", "y", "z", "roll", "pitch", "yaw"]
FILE_LOC = "~/Desktop/Fishberg UROP Su24/uropsu24/Metronome_Test_Data/"


def create_csv(rosbag_file):
    b = bagreader(rosbag_file)

    csvfiles = []
    for t in b.topics:
        data = b.message_by_topic(t)
        csvfiles.append(data)

    # print(csvfiles[0])
    return csvfiles[0]
# data = pd.read_csv(csvfiles[0])



def create_roll_pitch_yaw_csv(csv_file_quaternion):
    data = pd.read_csv(csv_file_quaternion)
    
    x = data["pose.position.x"]
    y = data["pose.position.y"]
    z = data["pose.position.z"]
    qx = data["pose.orientation.x"]
    qy = data["pose.orientation.y"]
    qz = data["pose.orientation.z"]
    qw = data["pose.orientation.w"]
    quat = np.column_stack((qx, qy, qz, qw))
    rot = Rot.from_quat(quat)
    # use ZYX [intrinsic yaw-pitch-roll seq] to get az-elev-rot3 data
    rot_seq = 'ZYX'
    rotations = np.rad2deg(rot.as_euler(rot_seq)) #from Andrew's script: 'zyx' uses extrinsic yaw-pitch-roll seq
    r1, r2, r3 = rotations[:, 0], rotations[:, 1], rotations[:, 2]

    
    # if rot_seq == 'ZYZ':
    #     DATA_TITLES = ["x", "y", "z", "rz3", "ry2", "rz1"]
    filename = csv_file_quaternion[:-4] + "-" + rot_seq + ".csv"
    FILE_PATH = FILE_LOC + filename

    # Create new csv file with x, y, z, roll, pitch, yaw
    df = pd.df = pd.DataFrame(columns=DATA_TITLES)  # Include the column titles)
    df.to_csv(FILE_PATH,  mode='a', header=True, index=False)
    
    # Add data to DataFrame object
    df = pd.DataFrame({
        DATA_TITLES[0]: x,  
        DATA_TITLES[1]: y,
        DATA_TITLES[2]: z,
        DATA_TITLES[3]: r3, #roll
        DATA_TITLES[4]: r2, #pitch (elevation)
        DATA_TITLES[5]: r1  #yaw (azimuth)
    }, columns=DATA_TITLES) # Include the column titles)
    df.to_csv(FILE_PATH,  mode='a', header=False, index=False)

def append_time(quat_csv, euler_csv):
    data1 = pd.read_csv(quat_csv)
    
    time_vals = data1["header.stamp.secs"]
    # example = time.strftime("%a, %d %b %Y %H:%M:%S +0000", time.gmtime(epoch))
    # print("Exanple feasible", example)

    
    data2 = pd.read_csv(euler_csv)
    x = data2['x']
    y = data2['y']
    z = data2['z']
    roll = data2['roll']
    pitch = data2['pitch']
    yaw = data2['yaw']
    
    FILE_PATH = FILE_LOC + euler_csv[:-4] + "-time.csv"
    
    # Create new csv file with x, y, z, roll, pitch, yaw
    mod_titles = ['time(sec)'] + DATA_TITLES
    df = pd.df = pd.DataFrame(columns=mod_titles)  # Include the column titles)
    df.to_csv(FILE_PATH,  mode='a', header=True, index=False)
    
    # Add data to DataFrame object
    df = pd.DataFrame({
        mod_titles[0]: time_vals,
        mod_titles[1]: x,  
        mod_titles[2]: y,
        mod_titles[3]: z,
        mod_titles[4]: roll,
        mod_titles[5]: pitch,
        mod_titles[6]: yaw
    }, columns=mod_titles) # Include the column titles)
    df.to_csv(FILE_PATH,  mode='a', header=False, index=False)
        

def get_time_segment(csv_file, time_init, time_final, new_filename):
    """
    Inputs:
        1) csv_file: expecting a title "header.stamp.secs"
        2) time_init: in sec
        3) time_final: in sec
        4) new_filename: file name for segmented data csv
    """
    df = pd.read_csv(csv_file)
    
    time_vals = df["time(sec)"]
    t0 = time_vals[0]
    tf = time_vals.iloc[-1]
    
    x = df['x']
    y = df['y']
    z = df['z']
    roll = df['roll']
    pitch = df['pitch']
    yaw = df['yaw']
    
    
    segment_inds = np.nonzero(np.logical_and(abs(time_vals - t0) >= time_init, abs(time_vals - t0) <= time_final))[0]
    print(type(segment_inds))
    segment_x = x[segment_inds]
    segment_y = y[segment_inds]
    segment_z = z[segment_inds]
    segment_roll = roll[segment_inds]
    segment_pitch = pitch[segment_inds]
    segment_yaw = yaw[segment_inds]
    segment_time = time_vals[segment_inds]
    
    FILE_PATH = FILE_LOC + new_filename
    
    # Create new csv file with x, y, z, roll, pitch, yaw
    mod_titles = ['time(sec)'] + DATA_TITLES
    df = pd.df = pd.DataFrame(columns=mod_titles)  # Include the column titles)
    df.to_csv(FILE_PATH,  mode='a', header=True, index=False)
    
    # Add data to DataFrame object
    df = pd.DataFrame({
        mod_titles[0]: segment_time,
        mod_titles[1]: segment_x,  
        mod_titles[2]: segment_y,
        mod_titles[3]: segment_z,
        mod_titles[4]: segment_roll,
        mod_titles[5]: segment_pitch,
        mod_titles[6]: segment_yaw
    }, columns=mod_titles) # Include the column titles)
    df.to_csv(FILE_PATH,  mode='a', header=False, index=False)


# Uncomment each section as needed
if __name__ == '__main__':
    print("Post process rosbag data")
    ## 1) Create csv files from bag files
    # parser = argparse.ArgumentParser()
    # parser.add_argument('rosbag_files',nargs='*')
    # args = parser.parse_args()
    # bag_files = args.rosbag_files
    
    ## Running py file in VsCode, no command-line args --> supply bag file names here
    # if len(bag_files) == 0:
    #     bag_files = ["fishberg_metv2/fishberg_met1v2_test123.bag", "fishberg_metv2/fishberg_met2v2_test123.bag" ]
    
    # for file in bag_files:
    #     create_csv(file)
    
    #-----------------------------------------------------------------------------
    
    ## 2) Convert quat to Euler angles
    # parser = argparse.ArgumentParser()
    # parser.add_argument('quat_csv_files',nargs='*')
    # args = parser.parse_args()
    # quat_csv_files = args.quat_csv_files
    
    ## Create csv files with Euler angle rotations
    ## Running py file in VsCode, no command-line args --> supply csv file names here
    # if len(quat_csv_files) == 0:
    #     quat_csv_files = ['metv3/ishberg_met1v2-world.csv', 'metv3/ishberg_met2v2-world.csv']
    # for quat in quat_csv_files:
    #     create_roll_pitch_yaw_csv(quat)

    #-----------------------------------------------------------------------------
   
    ## 3) Look at time info of csv files
    # euler_csv_files = ['fishberg_metv2/ishberg_met1v2-world-ZYX.csv', 'fishberg_metv2/ishberg_met2v2-world-ZYX.csv']
    # for quat, euler in zip(quat_csv_files, euler_csv_files):
        # create_roll_pitch_yaw_csv(file)
        # append_time(quat, euler)
        
        # data1 = pd.read_csv(quat)
        # time_vals = data1['header.stamp.secs']
        # t0 = time_vals[0]
        # tf = time_vals.iloc[-1]
        
        # # T_Initial = time.strftime("%a, %d %b %Y %H:%M:%S +0000", time.gmtime(t0/1e9))
        # print("Initial time", t0/60//60, 'hrs', t0//60, 'min', t0%60, 'sec')

        # # T_Final = time.strftime("%a, %d %b %Y %H:%M:%S +0000", time.gmtime(tf/1e9))
        # print("Final time", tf/60//60, 'hrs', tf//60, 'min', tf%60, 'sec')
        
        # print(f'Elapsed Time: {(tf-t0)//3600} hrs, {(tf-t0)//60} min, {(tf-t0)%60} sec')

    #-----------------------------------------------------------------------------

    ## 4) Create time-segmented csv files
    # file = "fishberg_metv2/ishberg_met2v2-world-ZYX-time.csv"
    # get_time_segment(file, 15*60+0, 22*60+34, "met2v2_seg3.csv")