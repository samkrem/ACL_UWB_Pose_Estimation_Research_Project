import pandas as pd
import numpy as np



file_path="~/Desktop/ACL_UROP/ACL_UWB_NN/Uniform(min=-0.1, max=0.1).csv"
data_titles=["pose_x", "pose_y", "pose z", "pose_roll", "pose_pitch", "pose_yaw", "z_true", "noise", "z"]
df = pd.DataFrame( columns=data_titles)  # Include the column titles)
df.to_csv(file_path,  mode='a', header=True, index=False)


x_coords=np.linspace(0,5,15)
y_coords=np.linspace(0,5,15)
z_coords=np.linspace(0,5,15)


roll_values = np.linspace(-180, 180, 10)
pitch_values = np.linspace(-90, 90, 10)
yaw_values = np.linspace(-180, 180, 10)

# roll_values = np.arange(-180, 180, 45)
# pitch_values = np.arange(-90, 90, 45)
# yaw_values = np.arange(-180, 180, 45)

poses=np.array([[x, y, z, roll, pitch, yaw] for x in x_coords for y in y_coords for z in z_coords for roll in roll_values for pitch in pitch_values for yaw in yaw_values])

def pose_z_noise_distr_creator(poses,noise):
    x_pose = [pose[0] for pose in poses ]
    y_pose = [pose[1] for pose in poses ]
    z_pose = [pose[2] for pose in poses ]
    roll = [pose[3] for pose in poses ]
    pitch = [pose[4] for pose in poses ]
    yaw = [pose[5] for pose in poses ]
    z_true=[np.linalg.norm(pose[:3]) for pose in poses ]

    z=np.subtract(z_true,noise)

    df = pd.DataFrame({data_titles[0]: x_pose,
    data_titles[1]: y_pose,
    data_titles[2]: z_pose,
    data_titles[3]: roll,
    data_titles[4]: pitch,
    data_titles[5]: yaw,
    data_titles[6]: z_true,
    data_titles[7]: noise,
    data_titles[8]: z} , 
    columns=data_titles)  # Include the column titles)
    df.to_csv(file_path,  mode='a', header=False, index=False)
size=poses.shape[0]
def create_trimodal_distribution(mean1, std1, mean2, std2, mean3, std3):
    # Bimodal Distribution
    n_samples = 72000


    # Generate the data
    data1 = np.random.normal(mean1, std1, n_samples)
    data2 = np.random.normal(mean2, std2, n_samples)
    data3 = np.random.normal(mean3, std3, n_samples)

    # Combine the data
    combined_data = np.concatenate([data1, data2, data3])
    return combined_data

#Normal(mean=0, stdev=1)

# loc1=0
# scale1=1
# noise_gaussian_1=np.random.normal(loc1,scale1, size= size) #mean, stdev, 
# distribution_noise_gaussian_1=f"Normal(mean=0,stdev=0.1)"
# pose_z_noise_distr_creator(poses, noise_gaussian_1, distribution_noise_gaussian_1)

# #Normal(mean=0, stdev=0.2)
# loc2=0
# scale2=0.2
# noise_gaussian_2=np.random.normal(loc2,scale2, size= size) #mean, stdev, 
# distribution_noise_gaussian_2=f"Normal(mean=0,stdev=0.2)"
# pose_z_noise_distr_creator(poses, noise_gaussian_2, distribution_noise_gaussian_2)

# #Uniform(min=-0.1, max=0.1)
min1=-0.1
max1=0.1
noise_uniform_1=np.random.uniform(min1,max1, size=size)
distribution_noise_uniform_1=f"Uniform(min=-0.1, max=0.1)"
pose_z_noise_distr_creator(poses, noise_uniform_1)

# #Uniform(min=-0.3, max=0.4)
# min2=-0.3
# max2=0.4
# noise_uniform_2=np.random.uniform(min2,max2, size=size)
# distribution_noise_uniform_2=f"Uniform(min=-0.3, max=0.4)"
# pose_z_noise_distr_creator(poses, noise_uniform_2, distribution_noise_uniform_1)

# #Binomial(n=100, p=0.5)
# n1=100
# p1=0.5
# noise_binomial_1=np.random.binomial(n1, p1, size=size)
# distribution_noise_binomial_1=f"Binomial(n=100, p=0.5)"
# pose_z_noise_distr_creator(poses, noise_binomial_1, distribution_noise_binomial_1)

# #Vonmises(mode=0, distribution dispersion=3)
# mu1=0
# kappa1=3
# noise_vonmises_1=np.random.vonmises(mu1, kappa1, size=size)
# distribution_noise_vonmises_1=f"Vonmises(mode=0, distribution dispersion=3)"
# pose_z_noise_distr_creator(poses, noise_vonmises_1, distribution_noise_vonmises_1)
# #Gamma(shape=100)
# shape1=100
# noise_gamma_1=np.random.standard_gamma(shape1, size=size)
# distribution_noise_gamma_1=f"Gamma(shape=100)"
# pose_z_noise_distr_creator(poses, noise_gamma_1, distribution_noise_gamma_1)
# #Gamma(shape=4)
# shape2=4
# noise_gamma_2=np.random.standard_gamma(shape2, size=size)
# distribution_noise_gamma_2=f"Gamma(shape=4)"
# pose_z_noise_distr_creator(poses, noise_gamma_2, distribution_noise_gamma_2)
# #Trimodal
# mean1, std1 = -10, 2
# mean2, std2 = 0, 4
# mean3, std3 = 10, 2
# noise_trimodal_1 = create_trimodal_distribution(mean1, std1, mean2, std2, mean3, std3)
# pose_z_noise_distr_creator(poses, noise_trimodal_1)