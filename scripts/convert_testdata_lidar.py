#!/usr/bin/env python3
import struct
import os
from pathlib import Path

DATA_DIR = Path('/home/lihongyu/Unicalib/UniCalib-OK/data/testdata')
BIN_DIR = DATA_DIR / 'Lidar' / 'data'
PCD_DIR = DATA_DIR / 'lidar_pcd'

BIN_DIR.mkdir(exist_ok=True)

def convert_bin_to_pcd(input_file, output_file):
    with open(input_file, 'rb') as f:
        data = f.read()

    n_points = len(data) // 24
    valid_points = []
    
    for i in range(n_points):
        x, y, z = struct.unpack('<fff', data[i*24:(i*24)+12])
        if abs(x) < 50 and abs(y) < 50 and abs(z) < 5:
            valid_points.append((x, y, z))

    with open(output_file, 'w') as f:
        f.write('VERSION .7\n')
        f.write('FIELDS x y z\n')
        f.write('SIZE 4 4 4\n')
        f.write('TYPE F F F\n')
        f.write('COUNT 1 1 1\n')
        f.write(f'WIDTH {len(valid_points)}\n')
        f.write('HEIGHT 1\n')
        f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write(f'POINTS {len(valid_points)}\n')
        f.write('DATA ascii\n')
        for x, y, z in valid_points:
            f.write(f'{x} {y} {z}\n')

bin_files = sorted(BIN_DIR.glob('*.bin'))
existing = set(p.stem for p in PCD_DIR.glob('*.pcd'))

print(f'Total bin files: {len(bin_files)}')
print(f'Existing pcd files: {len(existing)}')

converted = 0
for bin_file in bin_files:
    stem = bin_file.stem
    if stem in existing:
        continue
    
    pcd_file = PCD_DIR / f'{stem}.pcd'
    convert_bin_to_pcd(bin_file, pcd_file)
    converted += 1
    if converted % 500 == 0:
        print(f'Converted {converted} files...')

print(f'Done. Converted {converted} new files.')