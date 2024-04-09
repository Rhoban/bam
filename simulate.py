import numpy as np
from model import BaseModel, Model

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

    def step(self, volts: None | float, dt: float):
        """
        Steps the simulation for dt given the applied voltage.
        """
        gravity_torque = self.mass * g * self.length * np.sin(self.q)
        motor_torque = self.model.compute_motor_torque(volts, self.dq)
        friction_torque = self.model.compute_friction_torque(
            motor_torque, gravity_torque, self.dq
        )
        inertia = self.inertia + self.model.get_extra_inertia()
        net_torque = motor_torque + friction_torque + gravity_torque
        angular_acceleration = net_torque / inertia

        self.dq += angular_acceleration * dt
        self.q += self.dq * dt
        self.t += dt

    def rollout_log(self, log: dict, rereset : float = None):
        """
        Read a given log dict and return the sequential reached positions
        """
        positions = []

        t = 0.0
        dt = log["dt"]
        first_entry = log["entries"][0]
        self.reset(first_entry["position"], first_entry["speed"])

        for entry in log["entries"]:
            t += dt
            if rereset is not None and t > rereset:
                t = 0.
                self.reset(entry["position"], entry["speed"])
            positions.append(self.q)

            if entry["torque_enable"]:
                volts = entry["volts"]
            else:
                volts = None

            self.step(volts, dt)

        return positions

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

    model = Model()
    model.load_parameters("params.json")
    sim = Simulate1R(0.676, 0.105, model)
    sim.reset(-1.5, 0.0)
    while True:
        sim.step(None, 0.01)
        sim.draw()
        import time

        time.sleep(0.01)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                break
