import torch as th

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

    Outputs are 1D torch.Tensor of size 2 containing [tau_f_n, tau_m_n].
    Tau_f_n is the friction torque or the maximum friction torque depending on the model.
    """
    def __init__(self, 
                 window_size: int = 1,
                 hidden_dimension: int = 256, 
                 hidden_layers: int = 3, 
                 activation: th.nn.Module = th.nn.ReLU(),
                 last_layer_activation: th.nn.Module = th.nn.Identity(),
                 device: th.device = th.device("cpu")
                 ):
        super().__init__()

        self.device: th.device = device

        # Window size (kept as a parameter to be able to save and load the model)
        self.window_size = th.nn.Parameter(th.tensor(window_size), requires_grad=False)

        # MLP to compute the torque friction
        # The input is [dtheta_[n-w_s+1,n], tau_m_[n-w_s+1,n], tau_l_[n-w_s+1,n]]
        # The output is [tau_f]
        layers = [th.nn.Linear(3*window_size, hidden_dimension)]
        layers.append(activation)
        for _ in range(hidden_layers):
            layers.append(th.nn.Linear(hidden_dimension, hidden_dimension))
            layers.append(activation)
        layers.append(th.nn.Linear(hidden_dimension, 1))
        layers.append(last_layer_activation)

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
        
        if x.shape[1] != 4*self.window_size.item():
            raise ValueError(f"Input must be of size 4 * window_size = {4*self.window_size.item()}.")

        # Splitting the input
        volts = x[:, :self.window_size.item()]
        velocity = x[:, self.window_size.item():2*self.window_size.item()]
        torque_enable = x[:, 2*self.window_size.item():3*self.window_size.item()]
        tau_l = x[:, 3*self.window_size.item():]

        # Computing tau_m
        tau_m = torque_enable * ((self.kt / self.R) * volts - (self.kt**2) * velocity / self.R)

        # Determining the friction torque
        mlp_input = th.hstack([velocity, tau_m, tau_l])
        tau_f = self.friction_mlp(mlp_input)

        out = th.hstack([tau_f, tau_m[:, -1].unsqueeze(1)])
        if squeezed_input:
            out = out.squeeze(0)
        return out
    
    def save(self, filename: str) -> None:
        """
        Save the model.

        Args:   
            filename (str): file name
        """
        th.save(self, filename)

    @classmethod
    def load(cls, filename: str, device: th.device = th.device("cpu")) -> "FrictionNet":
        """
        Load the model.

        Args:
            filename (str): file name
        """
        friction_net = th.load(filename, map_location=device)
        friction_net.device = device
        return friction_net
    

if __name__ == "__main__":
    window_size = 1
    friction_net = FrictionNet(window_size)

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

    friction_net = FrictionNet(window_size=3, hidden_dimension=32, hidden_layers=2, activation=th.nn.Tanh())
    print("window_size before saving : ", friction_net.window_size.item())
    print("hidden_dimension before saving : ", friction_net.friction_mlp[0].out_features)

    friction_net.save(filename)
    friction_net_loaded = FrictionNet.load(filename)
    print("window_size after loading : ", friction_net_loaded.window_size.item())
    print("hidden_dimension after loading : ", friction_net_loaded.friction_mlp[0].out_features)

    os.remove(filename)
