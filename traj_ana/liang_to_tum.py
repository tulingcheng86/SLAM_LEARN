import re

def convert_to_tum(input_file, output_file):
    # 正则表达式，用于匹配并提取所需的数据
    timestamp_regex = re.compile(r'stamp:\s+sec: (\d+)\s+nanosec: (\d+)')
    position_regex = re.compile(r'position:\s+x: ([-\d.e]+)\s+y: ([-\d.e]+)\s+z: ([-\d.e]+)')
    orientation_regex = re.compile(r'orientation:\s+x: ([-\d.e]+)\s+y: ([-\d.e]+)\s+z: ([-\d.e]+)\s+w: ([-\d.e]+)')
    
    # 初始化变量
    poses = []

    with open(input_file, 'r') as infile:
        content = infile.read()

    # 分割文件内容为多个部分
    parts = content.split('---')
    for part in parts:
        timestamps = timestamp_regex.findall(part)
        positions = position_regex.findall(part)
        orientations = orientation_regex.findall(part)
        
        # 确保提取到的数据量一致
        if timestamps and len(positions) == len(orientations) and len(timestamps) >= len(positions):
            for i in range(len(positions)):
                sec, nanosec = timestamps[i]
                timestamp = float(sec) + float(nanosec) / 1e9
                x, y, z = positions[i]
                qx, qy, qz, qw = orientations[i]
                poses.append(f"{timestamp} {x} {y} {z} {qx} {qy} {qz} {qw}\n")

    with open(output_file, 'w') as outfile:
        outfile.writelines(poses)

# 替换为实际的输入文件和输出文件路径
input_file_path = 'issac_path.txt'
output_file_path = 'output_tum_format.txt'

convert_to_tum(input_file_path, output_file_path)
