import numpy as np
from model import BaseModel, load_model, load_network
from dynamixel import compute_volts

g: float = -9.81


class Simulate1R:
    def __init__(self, mass: float, length: float, model: BaseModel):
        # Mass [kg]
        self.mass: float = mass
        # Length [m]
        self.length: float = length

        self.screen = None
        self.model = model
        self.reset()

    def reset(self, q: float = 0.0, dq: float = 0.0):
        """
        Resets the simulation to a given state
        """
        self.q = q
        self.dq = dq
        self.t = 0.0

        self.inertia = self.mass * self.length**2

    def update_histories(self, volts: float, tau_m: float, tau_l: float, dq: float):
        self.volts_history.pop(0)
        self.volts_history.append(volts)
        self.tau_m_history.pop(0)
        self.tau_m_history.append(tau_m)
        self.tau_l_history.pop(0)
        self.tau_l_history.append(tau_l)
        self.dq_history.pop(0)
        self.dq_history.append(dq)

    def step(self, volts: None | float, dt: float):
        """
        Steps the simulation for dt given the applied voltage.
        """

        gravity_torque = self.mass * g * self.length * np.sin(self.q)
        motor_torque = self.model.compute_motor_torque(volts, self.dq)
        
        if self.model.network:
            self.update_histories(volts, motor_torque, gravity_torque, self.dq)
            frictionloss, damping = self.model.compute_frictions(self.volts_history, 
                                                                 self.tau_m_history, 
                                                                 self.tau_l_history,
                                                                 self.dq_history)
        else:
            frictionloss, damping = self.model.compute_frictions(motor_torque, 
                                                                 gravity_torque, 
                                                                 self.dq)

        inertia = self.inertia + self.model.get_extra_inertia()
        net_torque = motor_torque + gravity_torque

        # Tau_stop is the torque required to stop the motor (reach a velocity of 0 after dt)
        tau_stop = (inertia / dt) * self.dq + net_torque
        static_friction = -np.sign(tau_stop) * np.min(
            [np.abs(tau_stop), frictionloss + damping * np.abs(self.dq)]
        )
        net_torque += static_friction

        angular_acceleration = net_torque / inertia

        self.dq += angular_acceleration * dt
        self.dq = np.clip(self.dq, -100.0, 100.0)
        self.q += self.dq * dt
        self.t += dt

    def rollout_log(
        self, log: dict, reset_period: float = None, simulate_control: bool = False
    ):
        """
        Read a given log dict and return the sequential reached positions
        """
        positions = []
        all_volts = []

        reset_period_t = 0.0
        dt = log["dt"]
        id_first_entry = 0

        if self.model.network:
            positions = [entry["position"] for entry in log["entries"][:self.model.window_size-1]]
            all_volts = [entry["volts"] for entry in log["entries"][:self.model.window_size-1]]

            self.volts_history = [0.] + all_volts.copy()
            self.dq_history = [0.] + [entry["speed"] for entry in log["entries"][:self.model.window_size-1]]
            self.tau_m_history = [self.model.compute_motor_torque(volts, dq) for volts, dq in zip(self.volts_history, self.dq_history)]
            self.tau_l_history = [0.] + [self.mass * g * self.length * pos for pos in positions]
            
            id_first_entry = self.model.window_size - 1
        
        first_entry = log["entries"][id_first_entry]
        self.reset(first_entry["position"], first_entry["speed"])

        for i, entry in enumerate(log["entries"]):
            if i < id_first_entry:
                continue
            
            reset_period_t += dt
            if reset_period is not None and reset_period_t > reset_period:
                reset_period_t = 0.0
                self.reset(entry["position"], entry["speed"])
            positions.append(self.q)

            if entry["torque_enable"]:
                if simulate_control:
                    position_error = entry["goal_position"] - self.q
                    volts = compute_volts(position_error, log["kp"])
                else:
                    volts = entry["volts"]
            else:
                volts = None
            all_volts.append(volts)

            self.step(volts, dt)

        return positions, all_volts

    def draw(self):
        """
        Draw using pygame
        """
        import pygame

        if self.screen is None:
            pygame.init()
            self.screen = pygame.display.set_mode((800, 600))

        # Draw the background
        self.screen.fill((255, 255, 255))

        # Draw the pendulum
        x = 400 + 1000 * self.length * np.sin(self.q)
        y = 300 + 1000 * self.length * np.cos(self.q)
        pygame.draw.line(self.screen, (0, 0, 255), (400, 300), (x, y), 5)
        pygame.draw.circle(self.screen, (255, 0, 0), (int(x), int(y)), 10)
        pygame.display.flip()


if __name__ == "__main__":
    import pygame

    # model = load_network("models/106/tau_f_m/LeakyReLU-w1-n128-l1_loss.pth")
    model = load_model("params/m4.json")

    sim = Simulate1R(3.500, 0.105, model)
    sim.reset(-1.5, 0.0)
    while True:
        sim.step(-2.0, 0.01)
        sim.draw()
        import time

        time.sleep(0.01)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                break
