# 假设原始文件名为 'original.txt'，处理后的文件将保存为 'processed.txt'

# 读取原始文件
with open('vio_loop.txt', 'r', encoding='utf-8') as file:
    lines = file.readlines()

# 处理每一行，去掉行尾的最后一个空格（如果存在）
processed_lines = [line.rstrip(' \n') + '\n' for line in lines]

# 将处理后的内容写入新文件
with open('processed.txt', 'w', encoding='utf-8') as file:
    file.writelines(processed_lines)

print('处理完成，输出文件为 processed.txt')
