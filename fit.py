import argparse
from model import Model, BaseModel
import simulate
import logs

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--logdir", type=str, required=True)
args = arg_parser.parse_args()

model = Model()
logs = logs.Logs(args.logdir)

def compute_score(model: BaseModel, log: dict) -> float:
    mean_error = 0
    dt = log["dt"]
    simulator = simulate.Simulate1R(log["mass"], log["length"], model)
    first_entry = log["entries"][0]
    simulator.reset(first_entry["position"], first_entry["speed"])

    for entry in log["entries"]:
        error = abs(simulator.q - entry["position"])
        if entry["torque_enable"]:
            volts = entry["volts"]
        else:
            volts = None

        simulator.step(volts, dt)
        mean_error += error
    
    return mean_error / len(log["entries"])

def compute_scores(model: BaseModel):
    scores = 0
    for log in logs.logs:
        scores += compute_score(model, log)

    return scores / len(logs.logs)


    