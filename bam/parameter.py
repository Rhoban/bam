# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0


class Parameter:
    """A scalar model parameter with bounds and an optimization flag.

    Parameters are attached to a :class:`~bam.model.Model` by
    :meth:`~bam.model.Model.set_actuator` and collected by
    :meth:`~bam.model.Model.get_parameters` for optimization.

    :param value: Initial value.
    :param min: Lower bound used by the optimizer.
    :param max: Upper bound used by the optimizer.
    :param optimize: If ``False`` the parameter is held fixed during fitting.
    """

    def __init__(self, value: float, min: float, max: float, optimize: bool = True):
        self.value: float = value
        self.min: float = min
        self.max: float = max
        self.optimize: bool = optimize
