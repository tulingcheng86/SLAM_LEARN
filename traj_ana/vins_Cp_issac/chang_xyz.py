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

def transform_poses(poses, rotation_matrix):
    transformed_poses = poses.copy()
    positions = poses[:, 1:4]
    transformed_positions = positions.dot(rotation_matrix.T)
    transformed_poses[:, 1:4] = transformed_positions
    return transformed_poses

# 定义旋转矩阵，例如绕Z轴旋转90度
rotation_matrix = np.array([[0, -1, 0],
                            [1, 0, 0],
                            [0, 0, 1]])

# 加载和变换轨迹
poses_issac = load_tum_file('issac.txt')
transformed_poses_issac = transform_poses(poses_issac, rotation_matrix)
save_tum_file(transformed_poses_issac, 'transformed_issac.txt')

poses_vinsfusion = load_tum_file('vinsfusion.txt')
transformed_poses_vinsfusion = transform_poses(poses_vinsfusion, rotation_matrix)
save_tum_file(transformed_poses_vinsfusion, 'transformed_vinsfusion.txt')

