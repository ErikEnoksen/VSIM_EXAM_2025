import laspy
import numpy as np
from pathlib import Path

# Input LAZ files
input_files = [
    'C:/Users/erike/Documents/GitHub/VSIM_EXAM_2025/Lasdata/32-1-517-155-10.laz',
    'C:/Users/erike/Documents/GitHub/VSIM_EXAM_2025/Lasdata/32-1-517-154-07.laz',
    'C:/Users/erike/Documents/GitHub/VSIM_EXAM_2025/Lasdata/32-1-517-154-17.laz',
    'C:/Users/erike/Documents/GitHub/VSIM_EXAM_2025/Lasdata/32-1-517-155-00.laz',
    'C:/Users/erike/Documents/GitHub/VSIM_EXAM_2025/Lasdata/32-2-517-154-03.laz',
    'C:/Users/erike/Documents/GitHub/VSIM_EXAM_2025/Lasdata/32-2-517-155-00.laz'
]

output_file = 'lasdata.txt'

print("Reading and merging LAZ files...")

# Lists to store all points
all_x = []
all_y = []
all_z = []

# Read all files and collect points
for input_file in input_files:
    print(f"Reading {Path(input_file).name}...")
    las = laspy.read(input_file, laz_backend=laspy.LazBackend.Laszip)
    all_x.extend(las.x)
    all_y.extend(las.y)
    all_z.extend(las.z)

# Convert to numpy arrays for easier calculation
all_x = np.array(all_x)
all_y = np.array(all_y)
all_z = np.array(all_z)

print(f"\nTotal points: {len(all_x)}")

# Calculate center
center_x = (all_x.min() + all_x.max()) / 2
center_y = (all_y.min() + all_y.max()) / 2
center_z = (all_z.min() + all_z.max()) / 2

print(f"Original center: X={center_x:.2f}, Y={center_y:.2f}, Z={center_z:.2f}")

# Center around origin
all_x = all_x - center_x
all_y = all_y - center_y
all_z = all_z - center_z

print(f"Centered around origin")
print(f"New range: X=[{all_x.min():.2f}, {all_x.max():.2f}], Y=[{all_y.min():.2f}, {all_y.max():.2f}], Z=[{all_z.min():.2f}, {all_z.max():.2f}]")

# Write to ASCII file
print(f"\nWriting to {output_file}...")
with open(output_file, 'w') as f:
    # Write number of points at the top
    f.write(f"{len(all_x)}\n")
    # Write all points
    for x, y, z in zip(all_x, all_y, all_z):
        f.write(f"{x} {y} {z}\n")

print(f"Done! File saved as '{output_file}'")
