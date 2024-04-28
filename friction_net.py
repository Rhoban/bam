import torch as th
import numpy as np

class FrictionNet(th.nn.Module):
    """
    FrictionNet is a neural network that computes the friction torque given 
    the current and previous states of a motor. The motor armature, the motor resistance
    and the motor electrical torque constant are computed as learnable parameters. 
    
    Take as inputs 1D torch.Tensor of size 4 * window_size containing for the n-th entry:
    - volts_[n-w_s+1,n]
    - dtheta_[n-w_s+1,n]
    - torque_[n-w_s+1,n]
    - tau_l_[n-w_s+1,n]

    Outputs are 1D torch.Tensor of size 4 containing [tau_f_n + tau_m_n + tau_l_n, I_a, R, Kt]
    The loss should compare out[0] to (I_l + out[1]) * ddtheta_n.
    """
    def __init__(self, 
                 window_size: int = 1,
                 dt: int = 0.002,
                 hidden_dimension: int = 256, 
                 hidden_layers: int = 3, 
                 activation: th.nn.Module = th.nn.ReLU(),
                 device: th.device = th.device("cpu")
                 ):
        super().__init__()

        self.device: th.device = device
        self.window_size = window_size
        self.dt = dt

        # MLP to compute the torque friction
        # The input is [dtheta_[n-w_s+1,n], tau_m_[n-w_s+1,n], tau_l_[n-w_s+1,n]]
        # The output is [tau_f]
        layers = [th.nn.Linear(3*window_size, hidden_dimension)]
        layers.append(activation)
        for _ in range(hidden_layers):
            layers.append(th.nn.Linear(hidden_dimension, hidden_dimension))
            layers.append(activation)
        layers.append(th.nn.Linear(hidden_dimension, 1))

        self.friction_mlp = th.nn.Sequential(*layers).to(device)

        # Parameters to compute the motor torque (including back EMF)
        self.kt = th.nn.Parameter(th.tensor(2.0))
        self.R = th.nn.Parameter(th.tensor(2.0))
        self.I_a = th.nn.Parameter(th.tensor(0.01))

    def forward(self, x: th.Tensor) -> th.Tensor:
        if type(x) is not th.Tensor:
            raise ValueError("Input must be a torch.Tensor.")
        
        squeezed_input = False
        if x.dim() == 1:
            squeezed_input = True
            x = x.unsqueeze(0)
        
        if x.shape[1] != 4*self.window_size:
            raise ValueError(f"Input must be of size 4 * window_size = {4*self.window_size}.")

        # Splitting the input
        volts = x[:, :self.window_size]
        velocity = x[:, self.window_size:2*self.window_size]
        torque_enable = x[:, 2*self.window_size:3*self.window_size]
        tau_l = x[:, 3*self.window_size:]

        # Computing tau_m
        tau_m = torque_enable * ((self.kt / self.R) * volts - (self.kt**2) * velocity / self.R)

        # Determining the friction torque
        mlp_input = th.hstack([velocity, tau_m, tau_l])
        tau_f = self.friction_mlp(mlp_input).squeeze(1)
        
        # Returning the output
        out = th.hstack([(tau_f + tau_m[:, -1] + tau_l[:, -1]).unsqueeze(1),
                         self.I_a.repeat(x.shape[0], 1), 
                         self.R.repeat(x.shape[0], 1), 
                         self.kt.repeat(x.shape[0], 1)])
        
        if squeezed_input:
            out = out.squeeze(0)
        return out
    
    def save(self, filename: str) -> None:
        """
        Save the model.

        Args:   
            filename (str): file name
        """
        th.save(self.state_dict(), filename)

    @classmethod
    def load(cls, filename: str, device: th.device = th.device("cpu")) -> "FrictionNet":
        """
        Load the model from a file.

        Args:
            filename (str): file name
        """
        state_dict = th.load(filename, map_location=device)
        friction_net = cls()
        friction_net.load_state_dict(state_dict)

        return friction_net
    

class FrictionNetMax(FrictionNet):
    """
    FrictionNetMax is a neural network that computes the maximum friction torque given 
    the current and previous states of a motor. The motor armature, the motor resistance
    and the motor electrical torque constant are computed as learnable parameters. 
    
    Take as inputs 1D torch.Tensor of size 4 * window_size + 2 containing for the n-th entry:
    - volts_[n-w_s+1,n]
    - dtheta_[n-w_s+1,n]
    - torque_enable_[n-w_s+1,n]
    - tau_l_[n-w_s+1,n]
    - ddtheta_n
    - I_l

    Outputs are 1D torch.Tensor of size 4 containing [tau_m_n + tau_f_n - I_a * ddtheta_n, I_a, R, Kt]
    The loss should compare out[0] to I_l * ddtheta_n - tau_l_n.
    """
    def __init__(self, 
                 window_size: int = 1,
                 dt: int = 0.002,
                 hidden_dimension: int = 256, 
                 hidden_layers: int = 3, 
                 activation: th.nn.Module = th.nn.ReLU(),
                 last_layer_activation: th.nn.Module = th.nn.Softplus(),
                 device: th.device = th.device("cpu")
                 ):
        
        super().__init__()

        # MLP to compute the maximum torque friction
        # The input is [dtheta_[n-w_s+1,n], tau_m_[n-w_s+1,n], tau_l_[n-w_s+1,n]]
        # The output is [tau_f_max] and should be positive (which can be enforced with the last layer activation)
        layers = [th.nn.Linear(3*window_size, hidden_dimension)]
        layers.append(activation)
        for _ in range(hidden_layers):
            layers.append(th.nn.Linear(hidden_dimension, hidden_dimension))
            layers.append(activation)
        layers.append(th.nn.Linear(hidden_dimension, 1))
        layers.append(last_layer_activation)

        self.friction_mlp = th.nn.Sequential(*layers).to(device)

    def forward(self, x: th.Tensor) -> th.Tensor:
        if type(x) is not th.Tensor:
            raise ValueError("Input must be a torch.Tensor.")
        
        squeezed_input = False
        if x.dim() == 1:
            squeezed_input = True
            x = x.unsqueeze(0)
        
        if x.shape[1] != 4*self.window_size + 2:
            raise ValueError(f"Input must be of size 4 * window_size + 2 = {4*self.window_size + 2}.")

        # Splitting the input
        volts = x[:, :self.window_size]
        velocity = x[:, self.window_size:2*self.window_size]
        torque_enable = x[:, 2*self.window_size:3*self.window_size]
        tau_l = x[:, 3*self.window_size:4*self.window_size]
        ddtheta = x[:, -2]
        I_l = x[:, -1]

        # Computing tau_m
        tau_m = torque_enable * ((self.kt / self.R) * volts - (self.kt**2) * velocity / self.R)
        
        # Determining the friction torque
        mlp_input = th.hstack([velocity, tau_m, tau_l])
        tau_f_max = self.friction_mlp(mlp_input)

        tau_stop = -((I_l + self.I_a) * velocity[:, -1] / self.dt + tau_m[:, -1] + tau_l[:, -1]).unsqueeze(1)
        tau_f = th.sign(tau_stop) * th.min(th.abs(tau_stop), tau_f_max)

        # Returning the output
        out = th.hstack([(tau_m[:, -1] + tau_f.squeeze(1) - self.I_a * ddtheta).unsqueeze(1), 
                         self.I_a.repeat(x.shape[0], 1), 
                         self.R.repeat(x.shape[0], 1), 
                         self.kt.repeat(x.shape[0], 1)])
        
        if squeezed_input:
            out = out.squeeze(0)
        return out
    

if __name__ == "__main__":
    window_size = 1
    friction_net = FrictionNet(window_size)

    print("Testing FrictionNet :")
    x = th.randn((10, 4*window_size)).to(friction_net.device)
    y = friction_net(x)
    print("input shape : ", x.shape)
    print("output shape : ", y.shape)

    x = th.randn(4*window_size).to(friction_net.device)
    y = friction_net(x)
    print("input shape : ", x.shape)
    print("output shape : ", y.shape)

    filename = "test_friction_net.pth"
    friction_net.save(filename)
    friction_net_loaded = FrictionNet.load(filename)

    y_loaded = friction_net_loaded(x)

    print("output befor saving : ", y)
    print("output after loading : ", y_loaded)

    import os
    os.remove(filename)

    print("---------------------------------------")
    print("Testing FrictionNetMax :")
    friction_net_max = FrictionNetMax(window_size)
    x = th.randn((10, 4*window_size + 2)).to(friction_net_max.device)
    y = friction_net_max(x)
    print("input shape : ", x.shape)
    print("output shape : ", y.shape)

    x = th.randn(4*window_size + 2).to(friction_net_max.device)
    y = friction_net_max(x)
    print("input shape : ", x.shape)
    print("output shape : ", y.shape)