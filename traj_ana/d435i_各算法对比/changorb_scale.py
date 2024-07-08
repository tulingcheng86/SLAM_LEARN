import numpy as np

def load_tum_file(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
    poses = []
    for line in lines:
        if line[0].isdigit():
            data = line.strip().split()
            poses.append([float(x) for x in data])
    return np.array(poses)

def save_tum_file(poses, file_path):
    with open(file_path, 'w') as f:
        for pose in poses:
            f.write(" ".join(map(str, pose)) + "\n")

def scale_poses(poses, scale_factor):
    scaled_poses = poses.copy()
    scaled_poses[:, 1:4] *= scale_factor
    return scaled_poses

# Load and scale the large trajectory
poses_orbslam = load_tum_file('ORBslam3.txt')
scaled_poses_orbslam = scale_poses(poses_orbslam, 0.374)  # Adjust the scale factor as needed
save_tum_file(scaled_poses_orbslam, 'scaled_ORBslam3.txt')


