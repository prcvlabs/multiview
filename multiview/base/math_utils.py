
import math
import numpy as np

def cbrt(X):
    """The (signed) cube root of X."""
    return float(X) ** (1./3.) if 0 <= float(X) else -(-float(X)) ** (1./3.)

def finite_matrix(np_matrix):
    """Returns TRUE if _every_ element is finite (not inf, or nan) for
    the passed numpy matrix."""

    # Can be done as in one line for 2d matricies as follows:
    # np.logical_and.reduce(np.logical_and.reduce(np.isfinite(Z))):
    Z = np.isfinite(np_matrix)
    while len(Z.shape) > 0:
        Z = np.logical_and.reduce(Z)
    return Z

def clamp(num, min_val, max_val):
    """Clamps `min` within the range [`min_val`..`max_val`]."""
    return max(min_val, min(num, max_val))

def to_radians(deg):
    return deg * math.pi / 180.0

def to_percent(value, min_val, max_val):
    """
    Returns `value` as a percentage (actually a number in [0..1]) of the
    range [`min_val`..`max_val`].

    Example:
    
        # Returns 0.25
        to_percent(1.0, 0.0, 4.0)

    """
    return (clamp(value, min_val, max_val) - min_val) / (max_val - min_val)
    
