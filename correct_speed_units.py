import json
import argparse
import os
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("-i", "--input_dir", type=str, required=True)
parser.add_argument("-o", "--output_dir", type=str, required=True)
args = parser.parse_args()

os.makedirs(args.output_dir, exist_ok=True)

for filename in os.listdir(args.input_dir):
    file_path = os.path.join(args.input_dir, filename)
    print(f"Processing {file_path}")

    file = json.load(open(file_path))
    entries = file["entries"]
    for entry in entries:
        entry["speed"] = np.deg2rad(entry["speed"])

    with open(os.path.join(args.output_dir, filename), "w") as f:
        json.dump(file, f, indent=4)
