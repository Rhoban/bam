import numpy as np
from model import Model


class Simulator:
    def __init__(self, model: Model):
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

        self.model.reset()

    def step(self, control: None | float, dt: float):
        """
        Steps the simulation for dt given the applied control
        """
        bias_torque = self.model.actuator.testbench.compute_bias(self.q, self.dq)
        motor_torque = self.model.actuator.compute_torque(control, self.q, self.dq)
        frictionloss, damping = self.model.compute_frictions(
            motor_torque, bias_torque, self.dq
        )

        inertia = (
            self.model.actuator.testbench.compute_mass(self.q, self.dq)
            + self.model.actuator.get_extra_inertia()
        )
        net_torque = motor_torque + bias_torque

        # Tau_stop is the torque required to stop the motor (reach a velocity of 0 after dt)
        tau_stop = (inertia / dt) * self.dq + net_torque
        static_friction = -np.sign(tau_stop) * np.min(
            [np.abs(tau_stop), frictionloss + damping * np.abs(self.dq)]
        )
        net_torque += static_friction

        angular_acceleration = net_torque / inertia

        self.dq += angular_acceleration * dt
        self.dq = np.clip(self.dq, -100.0, 100.0)
        self.q += self.dq * dt + 0.5 * angular_acceleration * dt**2
        self.t += dt

    def rollout_log(
        self, log: dict, reset_period: float = None, simulate_control: bool = False
    ):
        """
        Read a given log dict and return the sequential reached positions
        """
        positions = []
        velocities = []
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
            velocities.append(self.dq)

            if entry["torque_enable"]:
                if simulate_control:
                    position_error = entry["goal_position"] - self.q
                    control = self.model.actuator.compute_control(
                        position_error, self.q, self.dq
                    )
                else:
                    if "control" in entry:
                        control = entry["control"]
                    else:
                        position_error = entry["goal_position"] - entry["position"]
                        control = self.model.actuator.compute_control(
                            position_error, self.q, self.dq
                        )
            else:
                control = None

            all_controls.append(control)

            self.step(control, dt)

        return positions, velocities, all_controls
