# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import numpy as np
import mujoco


class Pendulum:
    """MuJoCo counterpart of :class:`bam.testbench.Pendulum`.

    Builds a MuJoCo spec whose rigid-body dynamics match the analytic pendulum
    testbench: a point mass at the tip of a uniform rod driven by a single hinge.
    The zero angle is the arm pointing downward; positive angles are
    counter-clockwise (about the +x axis, with the default ``-z`` gravity).

    The body inertial is set explicitly so the inertia about the pivot equals

    .. math::

        M(q) = m\\,l^2 + \\frac{m_a}{3}\\,l^2

    and the gravity torque equals

    .. math::

        \\tau_e(q) = \\left(m + \\frac{m_a}{2}\\right) g\\,l\\,\\sin(q)

    exactly reproducing :meth:`bam.testbench.Pendulum.compute_mass` and
    :meth:`bam.testbench.Pendulum.compute_bias`.

    :param log: Log dict containing ``"mass"`` [kg], ``"arm_mass"`` [kg], and
        ``"length"`` [m] keys.
    """

    #: Gravity magnitude, matching :data:`bam.testbench` (g = -9.80665).
    G = 9.80665

    def __init__(self, log: dict):
        self.mass = log["mass"]
        self.arm_mass = log["arm_mass"]
        self.length = log["length"]

    def build_spec(self, name: str = "pendulum") -> mujoco.MjSpec:
        """Build and return a MuJoCo spec for this pendulum.

        :param name: Name given to both the hinge joint and the (motor) actuator.
            This is the name the :class:`~bam.mujoco.MujocoController` is created with.
        :returns: A :class:`mujoco.MjSpec` with a single hinge joint and a
            direct-torque (motor) actuator.
        """
        spec = mujoco.MjSpec()
        spec.option.gravity = [0.0, 0.0, -self.G]
        # One noslip solver iteration to reduce friction slip on the CPU pipeline.
        spec.option.noslip_iterations = 1
        # We set the inertial properties explicitly to match the analytic
        # testbench, so geoms are only visual.
        spec.compiler.inertiafromgeom = (
            mujoco.mjtInertiaFromGeom.mjINERTIAFROMGEOM_FALSE
        )

        length = self.length

        # ── Scenery (purely visual, no collisions so the pendulum dynamics are
        # identical to the analytic testbench) ──────────────────────────────
        spec.add_texture(
            name="skybox",
            type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
            builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
            width=256,
            height=256,
            rgb1=[0.3, 0.5, 0.7],
            rgb2=[0.0, 0.0, 0.0],
        )
        spec.add_texture(
            name="grid",
            type=mujoco.mjtTexture.mjTEXTURE_2D,
            builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
            width=300,
            height=300,
            rgb1=[0.2, 0.2, 0.2],
            rgb2=[0.3, 0.3, 0.3],
            mark=mujoco.mjtMark.mjMARK_EDGE,
            markrgb=[0.8, 0.8, 0.8],
        )
        floor_material = spec.add_material(
            name="grid", texrepeat=[5, 5], reflectance=0.2
        )
        floor_material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "grid"

        # A bit of light for some shadows.
        spec.worldbody.add_light(
            pos=[0.4, 0.4, 1.2], dir=[-0.4, -0.4, -1.2], castshadow=True
        )

        # Floor, placed just below the pendulum's lowest reach.
        floor = spec.worldbody.add_geom(
            name="floor",
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            pos=[0.0, 0.0, -(length + 0.12)],
            size=[2.0, 2.0, 0.1],
            material="grid",
            contype=0,
            conaffinity=0,
        )

        body = spec.worldbody.add_body(name=name, pos=[0.0, 0.0, 0.0])
        body.add_joint(
            name=name,
            type=mujoco.mjtJoint.mjJNT_HINGE,
            axis=[1.0, 0.0, 0.0],
        )

        total_mass = self.mass + self.arm_mass

        # Center of mass along the (downward) arm: rod contributes at l/2, the
        # tip mass at l.
        com_z = -(self.mass * length + self.arm_mass * length / 2.0) / total_mass

        # Inertia about the pivot (matches Pendulum.compute_mass), then shifted
        # to the center of mass with the parallel axis theorem for the spec.
        inertia_pivot = self.mass * length**2 + (self.arm_mass / 3.0) * length**2
        inertia_com = inertia_pivot - total_mass * com_z**2

        body.mass = total_mass
        body.ipos = [0.0, 0.0, com_z]
        # Only rotation about x (the hinge axis) matters for the 1-DOF dynamics,
        # and MuJoCo re-adds M*com_z^2 through the COM offset, so the hinge
        # inertia stays equal to inertia_pivot. We use an isotropic tensor with a
        # small floor: a point mass has zero inertia about its own COM, which
        # MuJoCo would reject.
        inertia_x = max(inertia_com, 1e-9)
        body.inertia = [inertia_x, inertia_x, inertia_x]

        # Visual arm: a brown stick (no mass/collision, inertia set explicitly).
        body.add_geom(
            name="arm",
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=[0.0, 0.0, 0.0, 0.0, 0.0, -length],
            size=[max(length * 0.025, 1e-3), 0.0, 0.0],
            mass=0.0,
            contype=0,
            conaffinity=0,
            rgba=[0.55, 0.35, 0.18, 1.0],
        )
        # Fake tip mass: a dark cylinder centered at the end of the arm.
        bob_half = max(length * 0.06, 1e-3)
        body.add_geom(
            name="bob",
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            fromto=[
                0.0, 0.0, -length + bob_half,
                0.0, 0.0, -length - bob_half,
            ],
            size=[max(length * 0.13, 2e-3), 0.0, 0.0],
            mass=0.0,
            contype=0,
            conaffinity=0,
            rgba=[0.13, 0.13, 0.15, 1.0],
        )

        # Direct-torque (motor) actuator on the hinge: ctrl is the joint torque.
        actuator = spec.add_actuator(name=name)
        actuator.target = name
        actuator.trntype = mujoco.mjtTrn.mjTRN_JOINT

        return spec


if __name__ == "__main__":
    import argparse
    import time
    import mujoco.viewer

    arg_parser = argparse.ArgumentParser(
        description="Open a MuJoCo viewer on a pendulum testbench (zero torque control)"
    )
    arg_parser.add_argument("--mass", type=float, default=0.5, help="Tip mass [kg]")
    arg_parser.add_argument("--arm_mass", type=float, default=0.1, help="Arm mass [kg]")
    arg_parser.add_argument("--length", type=float, default=0.2, help="Arm length [m]")
    arg_parser.add_argument(
        "--q0", type=float, default=1.0, help="Initial joint angle [rad]"
    )
    args = arg_parser.parse_args()

    pendulum = Pendulum(
        {"mass": args.mass, "arm_mass": args.arm_mass, "length": args.length}
    )
    model = pendulum.build_spec().compile()
    data = mujoco.MjData(model)
    data.qpos[0] = args.q0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            # Zero torque control: let gravity and friction act freely.
            data.ctrl[:] = 0.0
            mujoco.mj_step(model, data)
            viewer.sync()
            remaining = model.opt.timestep - (time.time() - step_start)
            if remaining > 0:
                time.sleep(remaining)
