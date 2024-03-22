import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons

# 读取数据，忽略最后的NaN列
data = pd.read_csv('/home/sbim/vo_path_with_timestamp.txt', delimiter=',', header=None, usecols=[0, 1, 2, 3])

# 提取时间戳和位置数据
timestamps = data[0]
x_positions = data[1]
y_positions = data[2]
z_positions = data[3]

# 创建绘图
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.2)

# 绘制X, Y, Z轴的数据
l0, = ax.plot(timestamps, x_positions, label='X Axis', color='r', visible=True)
l1, = ax.plot(timestamps, y_positions, label='Y Axis', color='g', visible=True)
l2, = ax.plot(timestamps, z_positions, label='Z Axis', color='b', visible=True)

# 添加图例
lines = [l0, l1, l2]
labels = ['X Axis', 'Y Axis', 'Z Axis']
visibility = [line.get_visible() for line in lines]
plt.legend(lines, labels)

# 添加复选按钮
rax = plt.axes([0.05, 0.4, 0.1, 0.15])
check = CheckButtons(rax, labels, visibility)

def func(label):
    index = labels.index(label)
    lines[index].set_visible(not lines[index].get_visible())
    plt.draw()

check.on_clicked(func)

plt.xlabel('Time')
plt.ylabel('Position')
plt.title('Position Change Over Time')
plt.grid(True)
plt.show()
