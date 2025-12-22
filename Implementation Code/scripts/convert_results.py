import csv
import sys
import os

# Usage: python3 convert_results.py input.csv output.tum

if len(sys.argv) < 3:
    print("Usage: python3 convert_results.py input.csv output.tum")
    sys.exit(1)

input_file = sys.argv[1]
output_file = sys.argv[2]

if not os.path.exists(input_file):
    print(f"Error: Input file {input_file} not found.")
    sys.exit(1)

print(f"Converting {input_file} to {output_file}...")

with open(input_file, 'r') as f_in, open(output_file, 'w') as f_out:
    reader = csv.reader(f_in, delimiter=',')
    
    count = 0
    for row in reader:
        # VINS: timestamp(ns), tx, ty, tz, qw, qx, qy, qz
        # TUM: timestamp(s) tx ty tz qx qy qz qw
        
        # Skip empty lines or lines with insufficient data
        if not row or len(row) < 8:
            continue

        try:
            # Handle potential trailing empty strings from csv reader if line ends with comma
            if row[-1] == '':
                row = row[:-1]

            timestamp_ns = float(row[0])
            timestamp_s = timestamp_ns / 1e9
            
            tx, ty, tz = row[1], row[2], row[3]
            qw, qx, qy, qz = row[4], row[5], row[6], row[7]
            
            # Reorder quaternion to qx qy qz qw and space separate
            f_out.write(f"{timestamp_s:.9f} {tx} {ty} {tz} {qx} {qy} {qz} {qw}\n")
            count += 1
        except ValueError:
            continue
        except IndexError:
            continue

print(f"Converted {count} poses.")
