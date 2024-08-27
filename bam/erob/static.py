import json
import time
import datetime
import os
import argparse
import erob.etherban as etherban

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--mass", type=float, required=True)
arg_parser.add_argument("--offset", type=float, required=True)
arg_parser.add_argument("--length", type=float, required=True)
arg_parser.add_argument("--port", type=str, default="/dev/ttyUSB0")
args = arg_parser.parse_args()

data = {}

def read_float(prompt):
    try:
        return float(input(prompt))
    except ValueError:
        return None

data["mass"] = args.mass
data["length"] = args.length
amps = read_float("Amps: ")

client = etherban.Client("localhost")
client.set_order(0, "torque", amps)
client.run_background()

has_value = False
for pos in "low", "high":
    if input(f"Snapshot the {pos} position [y/n] ? ") == "y":
        has_value = True
        status = client.get_statuses()[0]
        data[pos] = {
            "amps": status["current"],
            "position": status["position"] - args.offset
        }
    else:
        data[pos] = {
            "amps": None,
            "position": None
        }

# Slowly decreaing amps
while amps > 0:
    client.set_order(0, "torque", amps)
    amps = max(0, amps - 0.25)
    time.sleep(0.1)

if has_value:
    # Reading static.json if it exists
    if os.path.exists("static.json"):
        with open("static.json", "r") as f:
            static_data = json.load(f)
    else:
        static_data = []

    static_data.append(data)

    # Writing to static.json
    with open("static.json", "w") as f:
        json.dump(static_data, f)

client.stop()
