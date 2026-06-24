# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import numpy as np
from .testbench import Testbench
from bam.parameter import Parameter
from .message import yellow, print_parameter


class Actuator:
    """Abstract base class for all BAM actuator models.

    Subclasses implement the firmware control law and the motor torque equation
    for a specific actuator family.  An actuator is always attached to a
    :class:`~bam.model.Model` via :meth:`~bam.model.Model.set_actuator`, which
    calls :meth:`initialize` to create the motor parameters (``kt``, ``R``, …)
    on the model.

    :param testbench_class: Class used to instantiate the testbench when a log
        is loaded (typically :class:`~bam.testbench.Pendulum`).
    """

    def __init__(self, testbench_class: Testbench):
        self.testbench_class = testbench_class
        self.testbench: Testbench | None = None

    def set_model(self, model):
        """Attach this actuator to a model and run :meth:`initialize`.

        :param model: :class:`~bam.model.Model` instance to attach to.
        """
        self.model = model
        self.initialize()

    def reset(self):
        pass

    def load_log(self, log: dict):
        """Called when a log is loaded.

        Instantiates the testbench from the log metadata (mass, length, …)
        and updates actuator settings (kp, vin) from the log.

        :param log: Log dict as loaded from a processed JSON file.
        """
        self.testbench = self.testbench_class(log)

    def initialize(self):
        """Create motor parameters on the attached model.

        Called automatically by :meth:`set_model`. Must create at minimum
        ``self.model.kt`` and ``self.model.R`` as :class:`~bam.parameter.Parameter`
        instances.
        """
        raise NotImplementedError

    def control_unit(self) -> str:
        """Return the physical unit of the control signal (e.g. ``"volts"``, ``"amps"``)."""
        raise NotImplementedError

    def compute_control(
        self, q_target: float, q: float, dq: float, dt: float
    ) -> float | None:
        """Compute the control signal from the current state and target.

        :param q_target: Target joint angle [rad].
        :param q: Current joint angle [rad].
        :param dq: Current joint velocity [rad/s].
        :param dt: Timestep [s].
        :returns: Control signal in the unit given by :meth:`control_unit`.
        """
        raise NotImplementedError

    def compute_torque(
        self, control: float | None, torque_enable: bool, q: float, dq: float
    ) -> float:
        """Compute the motor torque from the control signal and current state.

        :param control: Control signal (volts, amps, or Nm depending on the actuator).
        :param torque_enable: Whether the actuator is powered.
        :param q: Current joint angle [rad].
        :param dq: Current joint velocity [rad/s].
        :returns: Motor torque [Nm].
        """
        raise NotImplementedError

    def get_extra_inertia(self) -> float:
        """Return the actuator's apparent inertia added to the load [kg·m²]."""
        raise NotImplementedError

    def to_mujoco(self):
        raise NotImplementedError("This actuator doesn't support to_mujoco")


class DCMotorActuator(Actuator):
    """Base class for DC motor actuators with a supply voltage and a P-gain.

    Loads ``kp`` and ``vin`` from the log metadata when :meth:`load_log` is called.

    :param testbench_class: Testbench class (see :class:`Actuator`).
    :param vin: Default supply voltage [V].
    :param kp: Default firmware proportional gain.
    """

    def __init__(
        self,
        testbench_class: Testbench,
        vin: float,
        kp: float,
    ):
        super().__init__(testbench_class)
        self.vin = vin
        self.kp = kp

    def load_log(self, log: dict):
        super().load_log(log)
        self.kp = log["kp"]
        if "vin" in log:
            self.vin = log["vin"]

    def initialize(self):
        """Create ``kt`` and ``R`` parameters on the attached model."""
        # Torque constant [Nm/A] or [V/(rad/s)]
        self.model.kt = Parameter(1.0, 0.0, 10.0)

        # Motor resistance [Ohm]
        self.model.R = Parameter(1.0, 0.0, 10.0)


class VoltageControlledActuator(DCMotorActuator):
    """Voltage-controlled servo with a firmware P-position controller.

    Implements the control law
    ``duty_cycle = clip(error_gain * kp * Δq, -max_pwm, max_pwm)``
    and the DC motor torque equation with back-EMF.

    :param testbench_class: Testbench class (see :class:`Actuator`).
    :param vin: Supply voltage [V].
    :param kp: Firmware proportional gain.
    :param error_gain: Converts ``kp * Δq`` to a duty cycle in [−1, 1].
        Depends on the servo's internal encoder resolution and gain scaling.
    :param max_pwm: Maximum duty cycle magnitude (default 1.0).
    """

    def __init__(
        self,
        testbench_class: Testbench,
        vin: float,
        kp: float,
        error_gain: float,
        max_pwm: float = 1.0,
    ):
        super().__init__(testbench_class, vin, kp)
        self.error_gain = error_gain
        self.max_pwm = max_pwm

    def control_unit(self) -> str:
        return "volts"

    def compute_control(
        self, q_target: float, q: float, dq: float, dt: float
    ) -> float | None:
        """Compute the voltage command from position error.

        :param q_target: Target joint angle [rad].
        :param q: Current joint angle [rad].
        :param dq: Current joint velocity [rad/s] (unused here).
        :param dt: Timestep [s] (unused here).
        :returns: Voltage [V] sent to the motor.
        """
        duty_cycle = (q_target - q) * self.kp * self.error_gain
        duty_cycle = np.clip(duty_cycle, -self.max_pwm, self.max_pwm)

        return self.vin * duty_cycle

    def compute_torque(
        self, control: float | None, torque_enable: bool, q: float, dq: float
    ) -> float:
        """Compute motor torque using the DC motor equation with back-EMF.

        :math:`\\tau = k_t V / R - k_t^2 \\dot{q} / R`

        :param control: Voltage [V].
        :param torque_enable: If ``False``, returns zero torque.
        :param q: Current joint angle [rad] (unused here).
        :param dq: Current joint velocity [rad/s].
        :returns: Motor torque [Nm].
        """
        volts = control
        torque = self.model.kt.value * volts / self.model.R.value
        torque -= (self.model.kt.value**2) * dq / self.model.R.value
        return torque * torque_enable

    def to_mujoco(self):
        if self.vin == 0 or self.kp == 0:
            print(yellow(f"WARNING: kp or vin are not set"))

        kt = self.model.kt.value
        R = self.model.R.value

        kp = self.error_gain * self.kp * self.vin * self.max_pwm * kt / R
        damping = self.model.friction_viscous.value + kt**2 / R

        print_parameter("forcerange", self.vin * self.model.kt.value / R)
        print_parameter("armature", self.model.armature.value)
        print_parameter("kp", kp)
        print_parameter("damping", damping)
        print_parameter("frictionloss", self.model.friction_base.value)

        print("")


class CurrentControlledActuator(DCMotorActuator):
    """Current-controlled servo with a firmware P-position controller.

    The controller outputs a target current proportional to position error,
    clipped by both the voltage limits and a user-configurable current cap.

    :param testbench_class: Testbench class (see :class:`Actuator`).
    :param vin: Supply voltage [V], used to compute the current saturation.
    :param kp: Firmware proportional gain.
    :param error_gain: Converts ``kp * Δq`` to a target current [A].
    """

    def __init__(
        self, testbench_class: Testbench, vin: float, kp: float, error_gain: float
    ):
        super().__init__(testbench_class, vin, kp)
        self.error_gain = error_gain

    def control_unit(self) -> str:
        return "amps"

    def initialize(self):
        super().initialize()
        self.model.current_limit = Parameter(1.5, 0, 3)
        self.model.viscous_damping_with_torque = Parameter(0.0, 0.0, 0.1)

    def compute_control(
        self, q_target: float, q: float, dq: float, dt: float
    ) -> float | None:
        """Compute the current command from position error.

        Clips the P-controller output by both the back-EMF voltage limit and
        the ``current_limit`` parameter.

        :param q_target: Target joint angle [rad].
        :param q: Current joint angle [rad].
        :param dq: Current joint velocity [rad/s].
        :param dt: Timestep [s] (unused here).
        :returns: Target current [A].
        """
        # Target current using simple P controller
        current = (q_target - q) * self.kp * self.error_gain

        # Maximum allowable current due to voltage limits
        current_limit_low = (1 / self.model.R.value) * (
            self.vin - self.model.kt.value * dq
        )
        current_limit_high = (1 / self.model.R.value) * (
            -self.vin - self.model.kt.value * dq
        )
        current = np.clip(current, current_limit_high, current_limit_low)

        # Maximum current allowed by the user to avoid heating
        current = np.clip(
            current, -self.model.current_limit.value, self.model.current_limit.value
        )

        return current

    def compute_torque(
        self, control: float | None, torque_enable: bool, q: float, dq: float
    ) -> float:
        """Compute motor torque from current command.

        :math:`\\tau = k_t I - b_{\\text{active}} \\dot{q}`

        :param control: Current command [A].
        :param torque_enable: If ``False``, returns zero torque.
        :param q: Current joint angle [rad] (unused here).
        :param dq: Current joint velocity [rad/s].
        :returns: Motor torque [Nm].
        """
        torque = (
            self.model.kt.value * control
            - self.model.viscous_damping_with_torque.value * dq
        )
        return torque * torque_enable

    def to_mujoco(self):
        if self.vin == 0 or self.kp == 0:
            print(yellow(f"WARNING: kp or vin are not set"))

        kt = self.model.kt.value

        kp = self.error_gain * self.kp * kt
        damping = (
            self.model.friction_viscous.value
            + self.model.viscous_damping_with_torque.value
        )
        
        forcerange = self.vin * self.model.kt.value / self.model.R.value
        forcerange = min(forcerange, self.model.current_limit.value * self.model.kt.value)

        print_parameter("forcerange", forcerange)
        print_parameter("armature", self.model.armature.value)
        print_parameter("kp", kp)
        print_parameter("damping", damping)
        print_parameter("frictionloss", self.model.friction_base.value)

        print("")
