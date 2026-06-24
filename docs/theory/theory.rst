Theoretical Framework
=====================

Pendulum test bench dynamics
----------------------------

For a single-axis pendulum test bench:

.. math::

   \tau_m + \tau_e(\theta) + \tau_f = J\ddot{\theta}

where:

- :math:`\theta` is the joint position
- :math:`\tau_m` is the actuator torque
- :math:`\tau_e(\theta) = mgl\sin(\theta)` is the gravity torque
- :math:`\tau_f` is the friction torque
- :math:`J` is the apparent inertia (load + actuator side)

Friction as a torque budget
---------------------------

The Coulomb-Viscous form is typically written as a force opposed to velocity.
In BAM, friction is handled through a maximum available resistive torque
:math:`\tau_{fm}`.

For a discrete simulator with time step :math:`\Delta t`, stopping at next step
requires:

.. math::

   \dot{\theta}_{k+1} = 0

which yields the stopping torque:

.. math::

   \tau_{f,stop} = -\left(\frac{J}{\Delta t}\dot{\theta} + \tau_m + \tau_e\right)

The applied friction is then this stopping torque clipped to the available friction budget:

.. math::

   \tau_f = \mathrm{clip}(\tau_{f,stop}, -\tau_{fm}, \tau_{fm})

Each model :math:`\mathcal{M}_i` defines :math:`\tau_{fm}` with a different level
of expressiveness.

Servo model + friction model coupling
-------------------------------------

In BAM, simulation combines:

- a servo model :math:`\mathcal{S}` that maps state and target to motor torque
- a friction model :math:`\mathcal{M}` that computes the friction budget

At each step:

1. compute :math:`\tau_m` from control law and motor model
2. compute :math:`\tau_{fm}` from chosen friction model
3. compute :math:`\tau_f` by clipping :math:`\tau_{f,stop}`
4. integrate acceleration from bench dynamics

This makes the framework both identifiable from logs and directly usable in
simulation.
