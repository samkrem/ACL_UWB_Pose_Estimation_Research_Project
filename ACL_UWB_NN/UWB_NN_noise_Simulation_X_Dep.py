import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation
import itertools
FILE_PATH="~/Desktop/ACL_UROP/ACL_UWB_NN/Normal(mean=0, stdev=f(stdev_roll,stdev_pitch,stdev_yaw)), distrib=stdev_roll+stdev_pitch+stdev_yaw"
DATA_TITLES=["pose_x", "pose_y", "pose z", "pose_roll", "pose_pitch", "pose_yaw", "z_true", "noise", "z", "distribution type"]

def pose_z_noise_distr_creator(poses,noise, distribution):    
    x_pose = [pose[0] for pose in poses ]
    y_pose = [pose[1] for pose in poses ]
    z_pose = [pose[2] for pose in poses ]
    
    roll = [pose[3] for pose in poses ]
    pitch = [pose[4] for pose in poses ]
    yaw = [pose[5] for pose in poses ]
    
    z_true=[np.linalg.norm(pose[:3]) for pose in poses ]
    z=np.add(z_true,noise)
    distribution=[distribution for pose in poses]

    df = pd.DataFrame({DATA_TITLES[0]: x_pose,
    DATA_TITLES[1]: y_pose,
    DATA_TITLES[2]: z_pose,
    DATA_TITLES[3]: roll,
    DATA_TITLES[4]: pitch,
    DATA_TITLES[5]: yaw,
    DATA_TITLES[6]: z_true,
    DATA_TITLES[7]: noise,
    DATA_TITLES[8]: z,
    DATA_TITLES[9]: distribution} , 
    columns=DATA_TITLES)  # Include the column titles)
    
    df.to_csv(FILE_PATH,  mode='a', header=False, index=False)

def euler_to_quaternion(angles):
    r = Rotation.from_euler('xyz', angles, degrees=True)
    return r.as_quat()
def quaternion_to_euler(quaternion):
    r = Rotation.from_quat(quaternion)
    euler = r.as_euler('xyz', degrees=True)
    return euler
def generate_x_dep_noise(poses, distribution_specs):
    max_dist=np.linalg.norm([4,4,4])
    mean=0
    
    #Normal Distribution, longer distance --> larger standard deviation (more variation of data)
    if distribution_specs == "Normal(mean=0,stdev=0.1*mag(<x,y,z>))":
        # distribution=[]
        # for i in range(len(poses)):
        #     stdev=0.1*np.linalg.norm(poses[i][:3])
        #     distribution.append(np.random.normal(mean,stdev))
        # distribution=np.array(distribution)

        stdevs= [2*np.linalg.norm(pose[:3]) for pose in poses] #[0,0, 1(100x))
        distribution=np.array([np.random.normal(mean, stdev) for stdev in stdevs]) #
        # stdev=1
        # means=[2*np.linalg.norm(pose[:3]) for pose in poses]
        # distribution=np.array([np.random.normal(mean, stdev) for mean in means])
        # distribution= [np.linalg.norm(pose[:3])+np.linalg.norm(pose[:3])*np.random.normal(0, 1) for pose in poses]

    #Normal Distribution, shorter distance --> larger standard distribution
    elif distribution_specs == "Normal(mean=0,stdev=0.1(max_dist-mag(<x,y,z>))":  
        stdevs= [0.1*(max_dist-np.linalg.norm(pose[:3])) for pose in poses]
        distribution=np.array([np.random.normal(mean, stdev) for stdev in stdevs])
    #Normal Distribution, larger z --> larger distribution of data
    elif distribution_specs == "Normal(mean=0,stdev=0.1*z)":
        stdevs= [3*pose[2] for pose in poses] 
        distribution=np.array([np.random.normal(mean, stdev) for stdev in stdevs])
    elif distribution_specs == "Normal(mean=0, stdev=0.1*pitch)":
        stdevs= [0.1*abs(pose[4]) for pose in poses]
        distribution=np.array([np.random.normal(mean, stdev) for stdev in stdevs])
    elif distribution_specs== "Normal(mean=0, stdev=0.1*el)": #el is arctan(z/x)
        stdevs=[]
        for pose in poses:
            if(np.linalg.norm(pose[:2])==0 and pose[2] !=0):
                stdevs.append(0.1*90)
            elif (np.linalg.norm(pose[:2])==0 and pose[2] ==0):
                stdevs.append(0)
            else:
                stdevs.append(.1*(180/np.pi)*np.arctan(pose[2]/np.linalg.norm(pose[:2])))
        distribution=np.array([np.random.normal(mean,stdev) for stdev in stdevs])
    elif distribution_specs == "Normal(mean=0, stdev=0.1*az)":
        stdevs=[]
        for pose in poses:
            if (pose[0]==0 and pose[1] !=0) or (pose[1]==0 and pose[0] !=0):
                stdevs.append(0.1*90)
            elif (np.linalg.norm(pose[0])==0 and pose[1] ==0):
                stdevs.append(0)
            else:
                stdevs.append(0.1*(180/np.pi)*np.arctan(pose[1]/pose[0]))
        distribution=np.array([np.random.normal(mean,stdev) for stdev in stdevs])
    elif distribution_specs == "Normal(mean=0, stdev=f(stdev_roll,stdev_pitch,stdev_yaw)), distrib=stdev_roll+stdev_pitch+stdev_yaw":
        stdevs_roll=[abs(0.1*pose[3]) for pose in poses]
        stdevs_pitch=[abs(0.1*pose[4]) for pose in poses]
        stdevs_yaw=[abs(0.1*pose[5]) for pose in poses]
       
        distribution_roll=np.array([np.random.normal(mean, stdev_roll) for stdev_roll in stdevs_roll])
        distribution_pitch=np.array([np.random.normal(mean, stdev_pitch) for stdev_pitch in stdevs_pitch])
        distribution_yaw=np.array([np.random.normal(mean, stdev_yaw) for stdev_yaw in stdevs_yaw])
     
        distribution= np.array([distr_roll_elem + distr_pitch_elem + distr_yaw_elem for distr_roll_elem, distr_pitch_elem, distr_yaw_elem in zip(distribution_roll, distribution_pitch, distribution_yaw)])
    elif distribution_specs == "Normal(mean=0, stdev=stdev(pitch)+ stdev(roll)+stdev(yaw))":
        stdevs=[abs(0.01*(pose[3]+pose[4]+pose[5])) for pose in poses]
        distribution=np.array([np.random.normal(mean, stdev) for stdev in stdevs])
    
    return distribution


#Rotation
def generate_poses():
    roll_values = np.arange(-180, 180, 30)
    pitch_values = np.arange(-90, 90, 30)
    yaw_values = np.arange(-180, 180, 30)

    euler_combinations = [[roll, pitch, yaw] for roll in roll_values for pitch in pitch_values for yaw in yaw_values]
    quaternion_combinations = [tuple(euler_to_quaternion(euler)) for euler in euler_combinations]

    # print(f"# Rotations: {len(quaternion_combinations)}")
    unique_quat_combo = set(quaternion_combinations)
    rotations=np.array([quaternion_to_euler(quat) for quat in unique_quat_combo])
    # print(f"# Unique Rotations: {rotations.shape[0]}")

    #Translation
    x_coords=np.arange(0,4,0.5)
    y_coords=np.arange(0,4,0.5)
    z_coords=np.arange(0,4,0.5)
    translations=np.array([[x, y, z] for x in x_coords for y in y_coords for z in z_coords])
    print(f"translations.shape: {translations.shape}")

    trans_rot_permutations=list(itertools.product(translations, rotations ))

    x_poses=np.array([np.concatenate([translations, rotations]) for translations, rotations in trans_rot_permutations]) #rename to x_poses
    print(x_poses)
    return x_poses

def main():
    x_poses=generate_poses()
    size=x_poses.shape[0]
    print(f"x_pose.shape: {x_poses.shape}")


    df = pd.DataFrame(columns=DATA_TITLES)  # Include the column titles)
    df.to_csv(FILE_PATH,  mode='a', header=True, index=False)
    #TRANSLATIONAL
    ## Normal(mean=0,stdev=0.1*mag(<x,y,z>))
    distribution_noise_gaussian_1=f"Normal(mean=0, stdev=f(stdev_roll,stdev_pitch,stdev_yaw)), distrib=stdev_roll+stdev_pitch+stdev_yaw"
    noise_gaussian_1=generate_x_dep_noise(x_poses, distribution_noise_gaussian_1)
    pose_z_noise_distr_creator(x_poses, noise_gaussian_1, distribution_noise_gaussian_1)

    # Normal(mean=0, stdev=0.2)
    # distribution_noise_gaussian_2=f"Normal(mean=0,stdev=0.1(max_dist-mag(<x,y,z>))" #rename
    # noise_gaussian_2=generate_x_dep_noise(x_poses, distribution_noise_gaussian_2)
    # pose_z_noise_distr_creator(x_poses, noise_gaussian_2, distribution_noise_gaussian_2)
   
    # ## Normal(mean=0,stdev=0.1*z)
    # distribution_noise_gaussian_3=f"Normal(mean=0,stdev=0.1*z)"
    # noise_gaussian_3=generate_x_dep_noise(x_poses, distribution_noise_gaussian_3)
    # pose_z_noise_distr_creator(x_poses, noise_gaussian_3, distribution_noise_gaussian_3)
    
    # #Normal(mean=0, stdev=0.1*el)
    # distribution_noise_gaussian_4=f"Normal(mean=0, stdev=0.1*el)"
    # noise_gaussian_4=generate_x_dep_noise(x_poses, distribution_noise_gaussian_4)
    # pose_z_noise_distr_creator(x_poses, noise_gaussian_4, distribution_noise_gaussian_4)

    # #Normal(mean=0, stdev=0.1*az)
    # distribution_noise_gaussian_5=f"Normal(mean=0, stdev=0.1*az)"
    # noise_gaussian_5=generate_x_dep_noise(x_poses, distribution_noise_gaussian_5)
    # pose_z_noise_distr_creator(x_poses, noise_gaussian_5, distribution_noise_gaussian_5)
    # #ROTATIONAL
    # ##Normal(mean=0, stdev=0.1*pitch)
    # distribution_noise_gaussian_6= f"Normal(mean=0, stdev=0.1*pitch)"
    # noise_gaussian_6=generate_x_dep_noise(x_poses, distribution_noise_gaussian_6)
    # pose_z_noise_distr_creator(x_poses, noise_gaussian_6, distribution_noise_gaussian_6)
    # ##Normal(mean=0, stdev=f(stdev_roll,stdev_pitch,stdev_yaw)), distrib=stdev_roll+stdev_pitch+stdev_yaw
    # distribution_noise_gaussian_7= f"Normal(mean=0, stdev=f(stdev_roll,stdev_pitch,stdev_yaw)), distrib=stdev_roll+stdev_pitch+stdev_yaw"
    # noise_gaussian_7=generate_x_dep_noise(x_poses, distribution_noise_gaussian_7)
    # pose_z_noise_distr_creator(x_poses, noise_gaussian_7, distribution_noise_gaussian_7)
  
    # ##Normal(mean=0, stdev=0.1((pitch)+ (roll)+ (yaw))
    # distribution_noise_gaussian_8=f"Normal(mean=0, stdev=stdev(pitch)+ stdev(roll)+stdev(yaw))"
    # noise_gaussian_8=generate_x_dep_noise(x_poses, distribution_noise_gaussian_8)
    # pose_z_noise_distr_creator(x_poses,noise_gaussian_8, distribution_noise_gaussian_8)

   








    ##
    ## Uniform(min=-0.1, max=0.1)
    # min1=-0.1
    # max1=0.1
    # noise_uniform_1=np.random.uniform(min1,max1, size=size)
    # distribution_noise_uniform_1=f"Uniform(min=-0.1, max=0.1)"
    # pose_z_noise_distr_creator(poses, noise_uniform_1, distribution_noise_uniform_1)


    ## Uniform(min=-0.3, max=0.4)
    # min2=-0.3
    # max2=0.4
    # noise_uniform_2=np.random.uniform(min2,max2, size=size)
    # distribution_noise_uniform_2=f"Uniform(min=-0.3, max=0.4)"
    # pose_z_noise_distr_creator(poses, noise_uniform_2, distribution_noise_uniform_1)


    ## Binomial(n=100, p=0.5)
    # n1=100
    # p1=0.5
    # noise_binomial_1=np.random.binomial(n1, p1, size=size)
    # distribution_noise_binomial_1=f"Binomial(n=100, p=0.5)"
    # pose_z_noise_distr_creator(poses, noise_binomial_1, distribution_noise_binomial_1,file_path, data_titles)


    ## Vonmises(mode=0, distribution dispersion=3)
    # mu1=0
    # kappa1=3
    # noise_vonmises_1=np.random.vonmises(mu1, kappa1, size=size)
    # distribution_noise_vonmises_1=f"Vonmises(mode=0, distribution dispersion=3)"
    # pose_z_noise_distr_creator(poses, noise_vonmises_1, distribution_noise_vonmises_1, file_path, data_titles)

    
    ## Gamma(shape=100)
    # shape1=100
    # noise_gamma_1=np.random.standard_gamma(shape1, size=size)
    # distribution_noise_gamma_1=f"Gamma(shape=100)"
    # pose_z_noise_distr_creator(poses, noise_gamma_1, distribution_noise_gamma_1, file_path, data_titles)

   
    ## Gamma(shape=4)
    # shape2=4
    # noise_gamma_2=np.random.standard_gamma(shape2, size=size)
    # distribution_noise_gamma_2=f"Gamma(shape=4)"
    # pose_z_noise_distr_creator(poses, noise_gamma_2, distribution_noise_gamma_2, file_path, data_titles)




if __name__ == "__main__":
    main()
