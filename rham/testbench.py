import numpy as np


class Testbench:
    """
    A Testbench is a physical setup used to test the motors

    The equation of motion of a Test Bench is given by:
    tau = M(q) * ddq + C(q, dq)

    where:
    - tau is the torque applied to the motor
    - M(q) is the mass matrix
    - ddq is the acceleration
    - C(q, dq) is the bias force
    """

    def compute_mass(self, q: float) -> float:
        """
        Compute the mass
        """
        raise NotImplementedError

    def compute_bias(self, q: float, dq: float) -> float:
        """
        Compute the bias force
        """
        raise NotImplementedError


class Pendulum:
    def __init__(self, log: dict):
        self.mass = log["mass"]
        self.arm_mass = log["arm_mass"]
        self.length = log["length"]

    def compute_mass(self, q: float, dq: float) -> float:
        """
        In the case of a pendulum, the mass is an inertia
        """
        inertia = self.mass * self.length**2
        inertia += (self.arm_mass / 3) * self.length**2

        return inertia

    def compute_bias(self, q: float, dq: float) -> float:
        g = -9.80665

        return (self.mass + self.arm_mass / 2) * g * self.length * np.sin(q)
    