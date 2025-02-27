from .model import models, load_model
import numpy as np
from .actuator import actuators
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--vin", type=float, default=5.1, help="VIN value")
parser.add_argument("--kp_firmware", type=int, default=1100, help="KP_FIRMWARE value")
parser.add_argument("--params", type=str, default="params.json")
args = parser.parse_args()

# TODO: Load model from JSON identified file
model = load_model(args.params)
# model = models["m1"]()
# model.set_actuator(actuators["xc330m288t"]())

viscous = model.friction_viscous.value
frictionloss = model.friction_base.value
# load_friction_base = model.load_friction_base.value
kt = model.kt.value
R = model.R.value
armature = model.armature.value

# From feetech
# R = 3.0
# kt = 0.78

# print(kt, R)
forcerange = args.vin * kt / R
kp = model.actuator.error_gain * args.kp_firmware * args.vin * kt / R
damping = viscous + kt**2 / R

print(f"Armature: {armature}")
print(f"Force range: {forcerange}")
print(f"Kp: {kp}")
print(f"Friction loss: {frictionloss}")
# print(f"Load friction base: {load_friction_base}")
print(f"Damping: {damping}")
