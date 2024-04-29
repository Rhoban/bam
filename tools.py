import torch as th

class CustomActivation(th.nn.Module):
    def forward(self, x):
        return th.where(th.abs(x) < 1, 0.5 * x ** 2, th.abs(x) - 0.5)

class AbsActivation(th.nn.Module):
    def forward(self, x):
        return th.abs(x)

class SquareActivation(th.nn.Module):
    def forward(self, x):
        return x ** 2

def get_activation(activation: str) -> th.nn.Module:
    if activation == "ReLU":
        return th.nn.ReLU()
    elif activation == "Tanh":
        return th.nn.Tanh()
    elif activation == "Softsign":
        return th.nn.Softsign()
    elif activation == "LeakyReLU":
        return th.nn.LeakyReLU()
    else:
        raise ValueError("Activation function not supported")
    
def get_last_activation(last: str) -> th.nn.Module:
    if last == "None":
        return th.nn.Identity()
    elif last == "Softplus":
        return th.nn.Softplus()
    elif last == "Abs":
        return AbsActivation()
    elif last == "Square":
        return SquareActivation()
    elif last == "Custom":
        return CustomActivation()
    else:
        raise ValueError(f"Last activation '{last}' function not supported")

def soft_min(x: th.Tensor, y: th.Tensor, beta: float = 0.9):
    K = beta * y
    return th.where(
        x < y,
        beta * x,
        K * (x - K) / (beta * x + y - 2 * K),
    )

def get_loss(loss: str) -> th.nn.Module:
    if loss == "smooth_l1_loss":
        return th.nn.functional.smooth_l1_loss
    elif loss == "l1_loss":
        return th.nn.functional.l1_loss
    elif loss == "mse_loss":
        return th.nn.functional.mse_loss
    else:
        raise ValueError("Loss function not supported")