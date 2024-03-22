import csv

# 替换为你的CSV文件路径
input_csv_file = 'MH_01-GT.csv'

# 输出的TUM格式文件路径
output_tum_file = 'output_tum_format.txt'

# 读取CSV文件并写入TUM格式
with open(input_csv_file, mode='r') as csv_file, open(output_tum_file, mode='w') as tum_file:
    csv_reader = csv.reader(csv_file)
    next(csv_reader)  # 跳过标题行
    for row in csv_reader:
        timestamp = int(row[0]) / 1e9  # 假设时间戳是纳秒，转换为秒
        x = row[1]
        y = row[2]
        z = row[3]
        qw = row[4]
        qx = row[5]
        qy = row[6]
        qz = row[7]
        
        # 将数据写入TUM格式
        tum_file.write(f"{timestamp} {x} {y} {z} {qx} {qy} {qz} {qw}\n")
