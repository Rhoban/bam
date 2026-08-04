# Copyright 2026 Theo Moore-Calters

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

from bam.feetech.actuator import STS3215Actuator
from bam.testbench import Testbench


class ST3025Actuator(STS3215Actuator):
    """Waveshare ST3025 servo operated at 12 V."""

    def __init__(self, testbench_class: Testbench):
        super().__init__(testbench_class)
        self.vin = 12.0
