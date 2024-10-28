"""
Export values from a model to a JSON file.

Usage:
python -m bam.fit \
    --actuator sts3250 \
    --model m6 \
    --logdir bam/data_processed_feetech \
    --method cmaes \
    --output params/feetech/m6.json \
    --trials 1000

python -m bam.export_values \
    --params params/feetech/m6.json \
    --vin 5.1 \
    --kp_firmware 32
"""

from .model import models, load_model
from .actuator import actuators
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--vin", type=float, default=5.1, help="VIN value")
parser.add_argument("--kp_firmware", type=int, default=32, help="KP_FIRMWARE value")
parser.add_argument("--params", type=str, default="params/feetech/m6.json")
args = parser.parse_args()

model = load_model(args.params)

viscous = model.friction_viscous.value
frictionloss = model.friction_base.value
kt = model.kt.value
R = model.R.value
armature = model.armature.value

forcerange = args.vin * kt / R
kp = model.actuator.error_gain * args.kp_firmware * args.vin * kt / R
damping = viscous + kt**2 / R


print(f"Armature: {armature}")
print(f"Force range: {forcerange}")
print(f"Kp: {kp}")
print(f"Friction loss: {frictionloss}")
print(f"Damping: {damping}")