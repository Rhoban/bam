import argparse
import numpy as np
import matplotlib.pyplot as plt
from model import load_model, DummyModel

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--params", type=str, default="params.json")
arg_parser.add_argument("--max_torque", type=float, required=True)
args = arg_parser.parse_args()

model = load_model(args.params)

torques = np.linspace(0, args.max_torque, 500)
lows = []
highs = []
for motor_torque in torques:
    external_torque_low = None
    external_torque_high = None
    for external_torque in torques:
        frictions, _ = model.compute_frictions(motor_torque, -external_torque, 0)

        if motor_torque - frictions > external_torque:
            if external_torque_low is None or external_torque > external_torque_low:
                external_torque_low = external_torque

        if external_torque_high is None and motor_torque + frictions < external_torque:
            external_torque_high = external_torque

    lows.append(external_torque_low)
    highs.append(external_torque_high)
        
plt.plot(torques, lows, color="blue", label="low")
plt.plot(torques, highs, color="red", label="high")
plt.grid()
plt.show()