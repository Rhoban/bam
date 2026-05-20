Friction Models (M1-M6)
=======================

BAM supports six actuator friction models of increasing expressiveness.

- M1: Coulomb-Viscous
- M2: Stribeck
- M3: Load-dependent
- M4: Stribeck + load-dependent
- M5: Directional load-dependent
- M6: Quadratic directional variant

Generic decomposition
---------------------

The friction term follows a decomposition of base friction and velocity-dependent damping:

.. math::

   \tau_f = \tau_{base}(\tau_m, \tau_{ext}, \dot{\theta}) + b\dot{\theta}

Where:\n
- :math:`\tau_m` is motor torque
- :math:`\tau_{ext}` is external/load torque
- :math:`\dot{\theta}` is joint velocity

Implementation reference
------------------------

Model behaviors are implemented in :mod:`bam.model`, notably:

- :class:`bam.model.Model`
- :func:`bam.model.load_model`
- :func:`bam.model.load_model_from_dict`
