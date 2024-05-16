import numpy as np
from model import BaseModel, Model, load_model

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
        self.model.reset()

    def step(self, control: None | float, dt: float):
        """
        Steps the simulation for dt given the applied control
        """
        gravity_torque = self.model.actuator.compute_gravity_torque(self.q, self.mass, self.length)
        motor_torque = self.model.actuator.compute_torque(control, self.q, self.dq)
        frictionloss, damping = self.model.compute_frictions(
            motor_torque, gravity_torque, self.dq
        )

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
        self.q += self.dq * dt + 0.5 * angular_acceleration * dt ** 2
        self.t += dt

    def rollout_log(
        self, log: dict, reset_period: float = None, simulate_control: bool = False
    ):
        """
        Read a given log dict and return the sequential reached positions
        """
        positions = []
        all_controls = []

        reset_period_t = 0.0
        dt = log["dt"]
        first_entry = log["entries"][0]
        self.reset(first_entry["position"], first_entry["speed"])
        self.model.actuator.load_log(log)

        for entry in log["entries"]:
            reset_period_t += dt
            if reset_period is not None and reset_period_t > reset_period:
                reset_period_t = 0.0
                self.reset(entry["position"], entry["speed"])
            positions.append(self.q)

            if entry["torque_enable"]:
                if simulate_control:
                    position_error = entry["goal_position"] - self.q
                    control = self.model.actuator.compute_control(position_error, self.q, self.dq)
                else:
                    if "control" in entry:
                        control = entry["control"]
                    else:
                        position_error = entry["goal_position"] - entry["position"]
                        control = self.model.actuator.compute_control(position_error, self.q, self.dq)
            else:
                control = None
                
            all_controls.append(control)

            self.step(control, dt)

        return positions, all_controls

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

    model = load_model("params.json")
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
