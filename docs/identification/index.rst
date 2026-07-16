Identification
==============

This section walks through the end-to-end identification pipeline: building
the test bench, recording trajectories, and fitting a friction model.

.. grid:: 1 2 2 2
    :gutter: 3
    :class-container: bam-getting-started

    .. grid-item-card:: 🔩 Hardware setup
        :link: setup
        :link-type: doc
        :class-card: bam-card

        Build the pendulum test bench with variable loads and lengths to collect
        a rich set of trajectories.

        +++
        :doc:`Hardware setup → <setup>`

    .. grid-item-card:: 🧩 Modeling the actuator
        :link: actuator_modeling
        :link-type: doc
        :class-card: bam-card

        Subclass BAM's :class:`~bam.actuator.Actuator` and implement the methods
        that describe your motor.

        +++
        :doc:`Actuator modeling → <actuator_modeling>`

    .. grid-item-card:: 🎬 Data acquisition
        :link: acquisition
        :link-type: doc
        :class-card: bam-card

        Run the actuator through predefined trajectories while logging position,
        velocity, and control signals.

        +++
        :doc:`Data acquisition → <acquisition>`

    .. grid-item-card:: 📉 Fitting
        :link: fitting
        :link-type: doc
        :class-card: bam-card

        Optimize the friction model parameters so the simulator reproduces the
        recorded trajectories.

        +++
        :doc:`Fitting → <fitting>`

    .. grid-item-card:: 🤝 Contributing a model
        :link: contributing
        :link-type: doc
        :class-card: bam-card

        Identified a new motor? Share your parameters with the community and add
        them to the BAM library.

        +++
        :doc:`Contributing → <contributing>`

.. toctree::
   :hidden:

   setup
   actuator_modeling
   acquisition
   fitting
   contributing
