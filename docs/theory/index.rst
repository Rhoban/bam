Theory
======

This section covers the theoretical foundations behind BAM: the pendulum
bench dynamics, the friction-budget formulation used in simulation, and the
six friction model variants M1–M6.

It follows the modeling and identification approach introduced
in the paper `"Extended Friction Models for the Physics Simulation of Servo
Actuators" <https://arxiv.org/pdf/2410.08650>`_ by Marc Duclusaud, Gregoire Passault, Vincent Padois, and Olivier
Ly .

.. grid:: 1 2 2 2
    :gutter: 3
    :class-container: bam-getting-started

    .. grid-item-card:: 📐 Theoretical framework
        :link: theory
        :link-type: doc
        :class-card: bam-card

        The pendulum test bench dynamics and the friction-budget formulation
        used to apply BAM models in simulation.

        +++
        :doc:`Theoretical framework → <theory>`

    .. grid-item-card:: 🧮 Friction models (M1–M6)
        :link: models
        :link-type: doc
        :class-card: bam-card

        The six friction models of increasing expressiveness, from Coulomb
        friction to load-dependent and Stribeck effects.

        +++
        :doc:`Friction models → <models>`

    .. grid-item-card:: ⚙️ DC motor
        :link: dc_motor
        :link-type: doc
        :class-card: bam-card

        How servomotor sources are modeled as DC motors, turning current into
        torque.

        +++
        :doc:`DC motor → <dc_motor>`

.. toctree::
   :hidden:

   theory
   models
   dc_motor
