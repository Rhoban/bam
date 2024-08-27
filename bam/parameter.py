
class Parameter:
    def __init__(self, value: float, min: float, max: float, optimize: bool = True):
        # Current value of the parameter
        self.value: float = value

        # Minimum and maximum values for the parameter
        self.min: float = min
        self.max: float = max

        # Should this parameter be optimized?
        self.optimize: bool = optimize