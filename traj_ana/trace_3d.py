import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# 使用pandas读取数据，并且只选择前8列
data = pd.read_csv('/home/sbim/vio_loop.txt', delimiter=',', header=None, usecols=[0, 1, 2, 3, 4, 5, 6, 7])

# 将数据转换为numpy数组用于分析
data_np = data.to_numpy()

# 提取时间戳、位置和四元数
timestamps = data_np[:, 0]
positions = data_np[:, 1:4]
quaternions = data_np[:, 4:8]

# 数据分析和可视化（如之前示例）
# 计算总位移
displacements = np.sqrt(np.sum(np.diff(positions, axis=0)**2, axis=1))
total_displacement = np.sum(displacements)
print(f"Total displacement: {total_displacement} units")

# 绘制三维轨迹图
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label='Trajectory')
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
plt.title('3D Trajectory')
plt.legend()
plt.show()
