import re

# 假定输入数据保存在这个文件中
input_file = 'data.txt'
# 输出的TUM格式文件路径
output_file = 'output.txt'

# 用于匹配时间戳和姿态信息的正则表达式
timestamp_regex = re.compile(r'stamp:\s+sec: (\d+)\s+nanosec: (\d+)')
position_regex = re.compile(r'position:\s+x: ([\d\.\-e]+)\s+y: ([\d\.\-e]+)\s+z: ([\d\.\-e]+)')
orientation_regex = re.compile(r'orientation:\s+x: ([\d\.\-e]+)\s+y: ([\d\.\-e]+)\s+z: ([\d\.\-e]+)\s+w: ([\d\.\-e]+)')

with open(input_file, 'r') as file, open(output_file, 'w') as out:
    content = file.read().split('---')  # 分割每一段数据
    for block in content:
        # 查找时间戳、位置和姿态
        timestamp_match = timestamp_regex.search(block)
        position_match = position_regex.search(block)
        orientation_match = orientation_regex.search(block)

        if timestamp_match and position_match and orientation_match:
            # 组合时间戳
            sec, nanosec = timestamp_match.groups()
            timestamp = float(sec) + float(nanosec) / 1e9
            # 提取位置和姿态
            x, y, z = position_match.groups()
            qx, qy, qz, qw = orientation_match.groups()

            # 写入TUM格式
            out.write(f"{timestamp} {x} {y} {z} {qx} {qy} {qz} {qw}\n")
