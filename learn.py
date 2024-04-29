import torch as th
import os
import wandb
import optparse
from torch.utils.data import DataLoader
from dataset import FrictionDataset
from friction_net import FrictionNet
from tools import get_activation, get_last_activation, get_loss, soft_min
from tqdm import tqdm

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
parser.add_option("--wandb", dest="wandb", default=0, type="int", help="using wandb")
parser.add_option("--max", action="store_true", help="use FrictionNetMax")
args = parser.parse_args()[0]

use_wandb = True if args.wandb == 1 else False

# Wandb initialization
if args.max:
    project_name = "friction-net-max"
    repository = "tau_f_m"
    model_name = args.activation + "-w" + str(args.window) + "-n" + str(args.nodes) + "-" + args.loss
    config = {"window": args.window, "nodes": args.nodes, "activation": args.activation, "last": args.last, "loss": args.loss}
else:
    project_name = "friction-net"
    repository = "tau_f"
    model_name = args.activation + "-w" + str(args.window) + "-n" + str(args.nodes) + "-" + args.loss
    config = {"window": args.window, "nodes": args.nodes, "activation": args.activation, "loss": args.loss}

device = th.device("cuda" if th.cuda.is_available() else "cpu")

# Load logs into dataset
dataset = FrictionDataset(window_size=args.window)
for log in os.listdir(args.data):
    dataset.add_log(os.path.join(args.data, log))

# Split dataset
dataset.shuffle()
train_dataset, test_dataset = dataset.split(0.8)

training_loader = DataLoader(train_dataset, batch_size=512, shuffle=False, drop_last=True, pin_memory=True if device == "cuda" else False)
testing_loader = DataLoader(test_dataset, batch_size=512, shuffle=False, drop_last=True, pin_memory=True if device == "cuda" else False)

# FrictionNet initialization
friction_net = FrictionNet(args.window, 
                        hidden_dimension=args.nodes, 
                        hidden_layers=3, 
                        activation=get_activation(args.activation),
                        last_layer_activation=get_last_activation(args.last),
                        device=device)

optimizer = th.optim.Adam(friction_net.parameters(), lr=1e-4, weight_decay=0)
scheduler = th.optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode="min", factor=0.5, patience=15, verbose=True, threshold=1e-3, threshold_mode="rel", cooldown=0, min_lr=1e-5, eps=1e-8)

# Loss function
loss_func = get_loss(args.loss)

def compute_loss(inputs, outputs, net):
    """
    Compute the loss of the network by comparing (tau_f + tau_m + tau_l) to (I_l + I_a) * ddtheta.
    Loss unit is Nm.
    """
    mlp_out = net(inputs)

    tau_m = mlp_out[:, 1]
    tau_l = inputs[:, -1]
    I_l = outputs[:, 1]
    I_a = th.abs(net.I_a)
    dtheta = inputs[:, 2*args.window-1]
    ddtheta = outputs[:, 0]

    if args.max:
        tau_f_max = mlp_out[:, 0]
        tau_stop = -((I_l + I_a) * dtheta / args.dt + tau_m + tau_l)

        tau_f = th.sign(tau_stop) * soft_min(th.abs(tau_f_max), th.abs(tau_stop))

    else:
        tau_f = mlp_out[:, 0]

    return loss_func(tau_f + tau_m + tau_l, (I_l + I_a) * ddtheta)
                                                         

# Training and testing functions
def train_epoch(net, loader):
    loss_sum = 0
    for batch in tqdm(loader):
        inputs = batch["input"].to(device)
        outputs = batch["output"].to(device)

        loss = compute_loss(inputs, outputs, net)
        loss_sum += loss.item()

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
    return loss_sum / len(loader)

def test_epoch(net, loader):
    loss_sum = 0
    for batch in tqdm(loader):
        inputs = batch["input"].to(device)
        outputs = batch["output"].to(device)

        loss_sum += compute_loss(inputs, outputs, net).item()
    return loss_sum / len(loader)


if use_wandb:
    wandb.init(project=project_name, name=model_name, config=config)
    wandb.watch(friction_net)

# Training loop
epochs = args.epochs
for epoch in range(epochs):
    print(f"Epoch {epoch + 1}/{epochs} ...")

    print("Train :")
    friction_net.train()
    avg_tloss = train_epoch(friction_net, training_loader)
    
    print("Testing :")
    friction_net.eval()
    with th.no_grad():
        avg_vloss = test_epoch(friction_net, testing_loader)

    if use_wandb:
        wandb.log({"avg_vloss": avg_vloss, 
                   "avg_tloss": avg_tloss,                   
                   "lr": optimizer.param_groups[0]["lr"], 
                   "epoch": epoch + 1,
                   "kt": friction_net.kt.item(),
                   "R": friction_net.R.item(),
                   "I_a": friction_net.I_a.item()})

    scheduler.step(avg_vloss)

    # if optimizer.param_groups[0]["lr"] <= 0.0004:
    #     friction_net.save("models/106/" + repository + "/" + model_name + ".pth")
    #     break
    
    # Saving the model
    if (epoch + 1) % 5 == 0:
        friction_net.save("models/106/" + repository + "/" + model_name + ".pth")