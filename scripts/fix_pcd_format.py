#!/usr/bin/env python3
import struct
from pathlib import Path

LIDAR_DIR = Path("/home/lihongyu/Unicalib/UniCalib-OK/data/drive_dusk/lidar")

def convert_pcd(input_file):
    with open(input_file, 'rb') as f:
        content = f.read()

    import re
    match = re.search(b'DATA binary\n', content)
    if not match:
        return
    
    data_start = match.end()
    data = content[data_start:]
    
    n_total = len(data)
    
    # 尝试用float(4字节)解析
    n_float = n_total // 4
    points_float = []
    for i in range(min(n_float, 1000)):
        try:
            x, y, z, intensity = struct.unpack('<ffff', data[i*16:(i+1)*16])
            if abs(x) < 100 and abs(y) < 100 and abs(z) < 20:
                points_float.append((x, y, z, intensity))
        except:
            pass
    
    # 尝试用double(8字节)解析
    n_double = n_total // 8
    points_double = []
    for i in range(min(n_double, 1000)):
        try:
            x, y, z, intensity = struct.unpack('<dddd', data[i*32:(i+1)*32])
            if abs(x) < 100 and abs(y) < 100 and abs(z) < 20:
                points_double.append((x, y, z, intensity))
        except:
            pass
    
    # 选择有效点多的格式
    if len(points_float) > len(points_double):
        points = points_float
        fmt = 'float'
    else:
        points = points_double
        fmt = 'double'
    
    if not points:
        print(f"  无法解析: {input_file.name}")
        return
    
    # 写入新文件 (ASCII格式)
    with open(input_file, 'w') as f:
        f.write('VERSION .7\n')
        f.write('FIELDS x y z intensity\n')
        f.write('SIZE 4 4 4 4\n')
        f.write('TYPE F F F F\n')
        f.write('COUNT 1 1 1 1\n')
        f.write(f'WIDTH {len(points)}\n')
        f.write('HEIGHT 1\n')
        f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write(f'POINTS {len(points)}\n')
        f.write('DATA ascii\n')
        for x, y, z, intensity in points:
            f.write(f'{x:.6f} {y:.6f} {z:.6f} {intensity:.6f}\n')

pcd_files = sorted(LIDAR_DIR.glob('*.pcd'))
print(f"转换 {len(pcd_files)} 个PCD文件...")

for i, pcd_file in enumerate(pcd_files):
    convert_pcd(pcd_file)
    if (i + 1) % 500 == 0:
        print(f"  已处理 {i+1}/{len(pcd_files)}")

print(f"完成! {len(pcd_files)} 个文件")

# 验证
print("\n验证转换结果:")
test_file = LIDAR_DIR / "00000000.pcd"
with open(test_file, 'r') as f:
    for _ in range(10):
        print(f.readline().strip())