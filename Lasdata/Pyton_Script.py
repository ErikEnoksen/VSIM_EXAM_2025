import laspy
import numpy as np
from pathlib import Path

# Input LAZ files - sorted by spatial position
input_files = [
    'C:/Users/erike/Documents/GitHub/VSIM_EXAM_2025/Lasdata/32-2-517-154-03.laz',
    'C:/Users/erike/Documents/GitHub/VSIM_EXAM_2025/Lasdata/32-1-517-154-07.laz',
    'C:/Users/erike/Documents/GitHub/VSIM_EXAM_2025/Lasdata/32-1-517-154-17.laz',
    'C:/Users/erike/Documents/GitHub/VSIM_EXAM_2025/Lasdata/32-1-517-155-00.laz',
    'C:/Users/erike/Documents/GitHub/VSIM_EXAM_2025/Lasdata/32-2-517-155-00.laz',
    'C:/Users/erike/Documents/GitHub/VSIM_EXAM_2025/Lasdata/32-1-517-155-10.laz',
]

output_file = 'lasdata.txt'

# CONFIGURATION
SCALE = 1.0  # Scale factor (1.0 = no scaling, 0.01 = 1/100 size)
SWAP_YZ = True  # Swap Y and Z so elevation becomes Y (up)
CENTER = True  # Center around origin
OFFSET_Y_TO_ZERO = True  # Move terrain so bottom is at Y=0

print("Reading and merging LAZ files...")

# Lists to store all points
all_x = []
all_y = []
all_z = []

# Read all files and collect points
for input_file in input_files:
    print(f"Reading {Path(input_file).name}...")
    las = laspy.read(input_file)
    all_x.extend(las.x)
    all_y.extend(las.y)
    all_z.extend(las.z)

# Convert to numpy arrays
all_x = np.array(all_x)
all_y = np.array(all_y)
all_z = np.array(all_z)

print(f"\nTotal points: {len(all_x)}")
print(f"\nOriginal data ranges:")
print(f"  X: {all_x.min():.2f} to {all_x.max():.2f}")
print(f"  Y: {all_y.min():.2f} to {all_y.max():.2f}")
print(f"  Z: {all_z.min():.2f} to {all_z.max():.2f} (elevation)")

# SWAP Y and Z if needed (Z elevation becomes Y up)
if SWAP_YZ:
    print(f"\nSwapping Y and Z (elevation becomes Y-up)...")
    temp = all_y.copy()
    all_y = all_z  # Z elevation becomes Y (up)
    all_z = temp   # Y north becomes Z (depth)
    print(f"After swap:")
    print(f"  X: {all_x.min():.2f} to {all_x.max():.2f}")
    print(f"  Y: {all_y.min():.2f} to {all_y.max():.2f}")
    print(f"  Z: {all_z.min():.2f} to {all_z.max():.2f}")

# CENTER around origin if needed
if CENTER:
    centerX = (all_x.min() + all_x.max()) / 2.0
    centerY = (all_y.min() + all_y.max()) / 2.0
    centerZ = (all_z.min() + all_z.max()) / 2.0
    
    print(f"\nCentering around origin...")
    print(f"  Center point: X={centerX:.2f}, Y={centerY:.2f}, Z={centerZ:.2f}")
    
    all_x = all_x - centerX
    all_y = all_y - centerY
    all_z = all_z - centerZ
    
    print(f"After centering:")
    print(f"  X: {all_x.min():.2f} to {all_x.max():.2f}")
    print(f"  Y: {all_y.min():.2f} to {all_y.max():.2f}")
    print(f"  Z: {all_z.min():.2f} to {all_z.max():.2f}")

# SCALE if needed
if SCALE != 1.0:
    all_x = all_x * SCALE
    all_y = all_y * SCALE
    all_z = all_z * SCALE
    print(f"After scaling:")
    print(f"  X: {all_x.min():.2f} to {all_x.max():.2f}")
    print(f"  Y: {all_y.min():.2f} to {all_y.max():.2f}")
    print(f"  Z: {all_z.min():.2f} to {all_z.max():.2f}")

# OFFSET Y so bottom is at Y=0 if needed
if OFFSET_Y_TO_ZERO:
    y_offset = -all_y.min()  # Move up by the minimum Y value
    print(f"\nOffsetting Y by {y_offset:.2f} to place bottom at Y=0")
    all_y = all_y + y_offset
    print(f"After Y offset:")
    print(f"  Y: {all_y.min():.2f} to {all_y.max():.2f}")

# Write to ASCII file
print(f"\nWriting to {output_file}...")
with open(output_file, 'w') as f:
    # Write number of points at the top
    f.write(f"{len(all_x)}\n")
    # Write all points
    for x, y, z in zip(all_x, all_y, all_z):
        f.write(f"{x} {y} {z}\n")

print(f"\nDone! File saved as '{output_file}'")
print(f"\nFinal configuration:")
print(f"  - Swap Y/Z: {SWAP_YZ}")
print(f"  - Centered: {CENTER}")
print(f"  - Scale: {SCALE}")
