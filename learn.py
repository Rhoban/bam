import torch as th
import wandb
import optparse
from torch.utils.data import DataLoader
from dataset import FrictionDataset
from friction_net import FrictionNet
from tqdm import tqdm

# Parse arguments
parser = optparse.OptionParser()
parser.add_option("--window", dest="window", default=1, type="int", help="window size")
parser.add_option("-n", "--nodes", dest="nodes", default=256, type="int", help="number of nodes per layer")
parser.add_option("-a", "--activation", dest="activation", default="ReLU", type="str", help="activation function")
parser.add_option("-e", "--epochs", dest="epochs", default=300, type="int", help="number of epochs")
parser.add_option("--loss", dest="loss", default="l1_loss", type="str", help="loss function")
parser.add_option("--last", dest="last", default="Abs", type="str", help="last layer activation function")
parser.add_option("--wandb", dest="wandb", default=0, type="int", help="using wandb")
parser.add_option("-m", "--max", action="store_true", help="use FrictionMaxNet")
args = parser.parse_args()[0]

use_wandb = True if args.wandb == 1 else False

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


train_dataset = FrictionDataset.load("datasets/106/" + repository + "/train_dataset_w" + str(args.window) + ".npz")
test_dataset = FrictionDataset.load("datasets/106/" + repository + "/test_dataset_w" + str(args.window) + ".npz")

# Data already shuffled in the datasets
training_loader = DataLoader(train_dataset, batch_size=512, shuffle=False, drop_last=True, pin_memory=True if device == "cuda" else False)
testing_loader = DataLoader(test_dataset, batch_size=512, shuffle=False, drop_last=True, pin_memory=True if device == "cuda" else False)

if args.activation == "ReLU":
    activation = th.nn.ReLU()
elif args.activation == "Tanh":
    activation = th.nn.Tanh()
elif args.activation == "Softsign":
    activation = th.nn.Softsign()
elif args.activation == "LeakyReLU":
    activation = th.nn.LeakyReLU()
else:
    raise ValueError("Activation function not supported")
    
friction_net = FrictionNet(args.window, 
                           hidden_dimension=args.nodes, 
                           hidden_layers=3, 
                           activation=activation,
                           device=device)

optimizer = th.optim.Adam(friction_net.parameters(), lr=1e-3, weight_decay=0)
scheduler = th.optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode="min", factor=0.5, patience=15, verbose=True, threshold=1e-3, threshold_mode="rel", cooldown=0, min_lr=1e-5, eps=1e-8)

if args.loss == "smooth_l1_loss":
    loss_func = th.nn.functional.smooth_l1_loss
elif args.loss == "l1_loss":
    loss_func = th.nn.functional.l1_loss
elif args.loss == "mse_loss":
    loss_func = th.nn.functional.mse_loss
else:
    raise ValueError("Loss function not supported")


def train_epoch(net, loader):
    loss_sum = 0
    for batch in tqdm(loader):
        inputs = batch["input"].to(device)
        outputs = batch["output"].to(device)

        mlp_out = net(inputs)
        loss = loss_func(mlp_out[:, 0], outputs[:, 0] * (outputs[:, 1] + mlp_out[:, 1]))
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

        mlp_out = net(inputs)
        loss = loss_func(mlp_out[:, 0], outputs[:, 0] * (outputs[:, 1] + mlp_out[:, 1]))
        loss_sum += loss.item()
    return loss_sum / len(loader)


if use_wandb:
    wandb.init(project=project_name, name=model_name, config=config)
    wandb.watch(friction_net)

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

    if optimizer.param_groups[0]["lr"] <= 0.0004:
        friction_net.save("models/106/tau_f/" + model_name + ".pth")
        break
    
    # Saving the model
    if (epoch + 1) % 5 == 0:
        friction_net.save("models/106/tau_f/" + model_name + ".pth")