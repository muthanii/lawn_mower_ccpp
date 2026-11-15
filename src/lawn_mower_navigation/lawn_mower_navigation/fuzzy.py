"""A small zero-order Sugeno fuzzy controller with two inputs.

Each input has triangular sets at given centres; the outer sets keep full
membership beyond the last centre. Every pair of sets is one rule with a
constant output, and the controller returns the weighted average of the rule
outputs.
"""

import numpy as np


def memberships(value, centres):
    """Degree of `value` in each triangular set. The degrees add up to 1."""
    centres = np.asarray(centres, dtype=float)
    out = np.zeros(len(centres))
    if value <= centres[0]:
        out[0] = 1.0
    elif value >= centres[-1]:
        out[-1] = 1.0
    else:
        hi = int(np.searchsorted(centres, value, side='right'))
        lo = hi - 1
        share = (value - centres[lo]) / (centres[hi] - centres[lo])
        out[lo], out[hi] = 1.0 - share, share
    return out


class FuzzyTable:
    """Rules: output[i][j] applies to set i of the first input and set j of the second."""

    def __init__(self, first_centres, second_centres, outputs):
        self.first = np.asarray(first_centres, dtype=float)
        self.second = np.asarray(second_centres, dtype=float)
        self.outputs = np.asarray(outputs, dtype=float)
        if self.outputs.shape != (len(self.first), len(self.second)):
            raise ValueError('outputs must have one row per first set and one column per second set')
        if np.any(np.diff(self.first) <= 0) or np.any(np.diff(self.second) <= 0):
            raise ValueError('set centres must increase')

    def __call__(self, first, second):
        return float(memberships(first, self.first) @ self.outputs @ memberships(second, self.second))
