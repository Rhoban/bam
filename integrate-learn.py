import torch as th
import os
import wandb
import optparse
import numpy as np
from dynamixel import MAX_PWM, ERROR_GAIN
from friction_net import FrictionNet
from dataset import IntegrateDataset
from tools import get_activation, get_last_activation, get_loss, soft_min
from tqdm import tqdm

USE_TQDM = True

# Parse arguments
parser = optparse.OptionParser()
parser.add_option("--window", dest="window", default=1, type="int", help="window size")
parser.add_option("--data", dest="data", default="data_106_network", type="str", help="data directory")
parser.add_option("--dt", dest="dt", default=0.002, type="float", help="log time step")
parser.add_option("--nodes", dest="nodes", default=256, type="int", help="number of nodes per layer")
parser.add_option("--activation", dest="activation", default="ReLU", type="str", help="activation function")
parser.add_option("--epochs", dest="epochs", default=300, type="int", help="number of epochs")
parser.add_option("--loss", dest="loss", default="l1_loss", type="str", help="loss function")
parser.add_option("--last", dest="last", default="Abs", type="str", help="last layer activation function")
parser.add_option("--wandb", action="store_true", default = False, help="use wandb")
parser.add_option("--max", action="store_true", default = False, help="use FrictionNetMax")
parser.add_option("--save", action="store_true", default = False, help="save the model")
args = parser.parse_args()[0]

# Wandb initialization
if args.wandb:
    project_name = "friction-net-integrate"
    
    repository = "integrate_tau_f" if not args.max else "integrate_tau_f_m"
    model_name = repository + "-" + args.activation + "-w" + str(args.window) + "-n" \
                 + str(args.nodes) + "-" + args.loss + "-" + args.last

    config = {"window": args.window, "nodes": args.nodes, "activation": args.activation, "loss": args.loss, "last": args.last, "max": args.max}

device = th.device("cuda" if th.cuda.is_available() else "cpu")

# Load logs
dataset = IntegrateDataset()
for log in os.listdir(args.data):
    dataset.add_log(os.path.join(args.data, log))

# Split dataset
dataset.shuffle()
train_dataset, test_dataset = dataset.split(0.65)

# FrictionNet initialization
friction_net = FrictionNet(args.window, 
                           hidden_dimension=args.nodes, 
                           hidden_layers=3, 
                           activation=get_activation(args.activation),
                           last_layer_activation=get_last_activation(args.last) if args.max else th.nn.Identity(),
                           simplify_tau_m=False,
                           device=device)

optimizer = th.optim.Adam(friction_net.parameters(), lr=1e-4, weight_decay=0)
scheduler = th.optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode="min", factor=0.5, patience=15, verbose=True, threshold=1e-3, threshold_mode="rel", cooldown=0, min_lr=1e-5, eps=1e-8)

# Loss function
loss_func = get_loss(args.loss)

error_gain = th.tensor(ERROR_GAIN).to(device)
max_pwm = th.tensor(MAX_PWM).to(device)

def compute_volts(position_error: th.tensor, kp: int, vin: np.float32 = 15.0):
    duty_cycle = position_error * kp * error_gain
    duty_cycle = th.clip(duty_cycle, -max_pwm, max_pwm)

    return vin * duty_cycle

def compute_tau_m(volts: th.tensor, dtheta: th.tensor, net):
    return net.kt * volts / net.R - (net.kt**2) * dtheta / net.R

def update_history(history: th.tensor, entry: th.tensor):
    history = history[1:]
    history = th.cat((history, entry))
    return history

def compute_loss(log, net):
    """
    Compute the loss of the network by comparing the integrated positions and the actual positions.
    """
    I_l = np.float32(log["mass"] * log["length"]**2)
    dt = np.float32(log["dt"])

    first_pos = th.tensor([np.float32(entry["position"]) for entry in log["entries"][:args.window]]).to(device)
    first_goal_pos = th.tensor([np.float32(entry["goal_position"]) for entry in log["entries"][:args.window]]).to(device)
    first_volts = compute_volts(first_goal_pos - first_pos, log["kp"])
    
    dtheta_history = th.tensor([np.float32(entry["speed"]) for entry in log["entries"][:args.window]]).to(device)
    tau_m_history = compute_tau_m(first_volts, dtheta_history, net)
    torque_enable_history = th.tensor([np.float32(entry["torque_enable"]) for entry in log["entries"][:args.window]]).to(device)
    tau_l_history = np.float32(-9.81 * log["mass"] * log["length"]) * first_pos

    loss = 0
    predicted_position = th.tensor([np.float32(log["entries"][args.window]["position"])]).to(device)
    for i, entry in enumerate(log["entries"]):
        if i < args.window:
            continue

        volts = compute_volts(th.tensor([np.float32(entry["goal_position"])]).to(device) - predicted_position, log["kp"])

        tau_m = torque_enable_history[-1] * compute_tau_m(volts, dtheta_history[-1], net)
        tau_l = np.float32(-9.81 * log["mass"] * log["length"]) * predicted_position
        
        mlp_input = th.hstack((dtheta_history, tau_m_history, torque_enable_history, tau_l_history)).to(device)        
        tau_f_max = net(mlp_input)[0]

        tau_stop = (I_l + friction_net.I_a) * dtheta_history[-1] / dt + tau_m + tau_l
        tau_f = -th.sign(tau_stop) * th.min(th.abs(tau_stop), th.abs(tau_f_max))

        dtheta = dtheta_history[-1] + (tau_f + tau_m + tau_l) / (I_l + friction_net.I_a) * dt
        dtheta = th.clip(dtheta, -100.0, 100.0)

        dtheta_history = update_history(dtheta_history, dtheta)
        tau_m_history = update_history(tau_m_history, tau_m)
        torque_enable_history = update_history(torque_enable_history, th.tensor([np.float32(entry["torque_enable"])]).to(device))
        tau_l_history = update_history(tau_l_history, tau_l)

        predicted_position = predicted_position + dtheta_history[-1] * dt
        loss += loss_func(th.tensor([np.float32(entry["position"])]).to(device), predicted_position)

    return loss / (len(log["entries"]) - args.window)


# Training and testing functions
def train_epoch(net, dataset):
    loss_sum = 0
    for log in tqdm(dataset.logs) if USE_TQDM else dataset.logs:
        loss = compute_loss(log, net)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        loss_sum += loss.item()
    return loss_sum / (len(log["entries"]) - args.window)

def test_epoch(net, dataset):
    loss_sum = 0
    for log in tqdm(dataset.logs) if USE_TQDM else dataset.logs:
        loss_sum += compute_loss(log, net)
    return loss_sum / (len(log["entries"]) - args.window)


if args.wandb:
    wandb.init(project=project_name, name=model_name, config=config)
    wandb.watch(friction_net)

# Training loop
epochs = args.epochs
for epoch in range(epochs):
    print(f"Epoch {epoch + 1}/{epochs} ...")

    print("Training...")
    friction_net.train()
    avg_tloss = train_epoch(friction_net, train_dataset)
    
    print("Testing...")
    friction_net.eval()
    with th.no_grad():
        avg_vloss = test_epoch(friction_net, test_dataset)

    if args.wandb:
        wandb.log({"avg_vloss": avg_vloss, 
                   "avg_tloss": avg_tloss,                   
                   "lr": optimizer.param_groups[0]["lr"], 
                   "epoch": epoch + 1,
                   "I_a": friction_net.I_a.item(),
                   "kt": friction_net.kt.item(),
                   "R": friction_net.R.item()})

    scheduler.step(avg_vloss)

    # if optimizer.param_groups[0]["lr"] <= 0.0004:
    #     friction_net.save("models/106/" + repository + "/" + model_name + ".pth")
    #     break
    
    # Saving the model
    if args.save:
        friction_net.save("models/106/" + repository + "/" + model_name + ".pth")