
import math
import numpy as np
import pdb
import functools
import random
import multiprocessing

# Python path added in ../directory
from base import *

# -------------------------------------------------------------------------------------- Cross Ratio

def cross_ratio(A, B, C, D):
    """The cross ration of four _colinear_ points is invariant under projective
    transformation. That means that, for any homography H,

         cross_ratio(A, B, C, D) == cross_ratio(HA, HB, HC, HD)

    which can be useful."""    

    # (u, v, w) is the line orthogonal to (A-D), that contains A
    u = A[0] - D[0]
    v = A[1] - D[1]
    w = -(u*A[0] + v*A[1])

    # Find the point-line distances
    a = u*A[0] + v*A[1] + w   # i.e., 0
    b = u*B[0] + v*B[1] + w   
    c = u*C[0] + v*C[1] + w   
    d = u*D[0] + v*D[1] + w
    
    return ((a - c)*(b - d)) / ((b - c) * (a - d))

# --------------------------------------------------------------------------------- Apply Homography

def apply_homography(H, x):
    """
    Applyies the homography 'H' to point 'x'. If 'x' has length
    two, then it is "lifted" into P2, and the result is normalized.
    If 'x' is an np.array, then an np.array is returned. Otherwise
    a list is returned.
    """
    l = len(x)
    v = x
    is_nd = isinstance(x, np.ndarray)
    if not is_nd:
        v = np.array(x, H.dtype)
    u = np.array(v, H.dtype)
    
    if l == 2:
        z_inv = 1.0 / (H[2,0:2].dot(v) + H[2,2])
        if(abs(z_inv) < 1e-20):
            z_inv = 1e-20
        u[0] = (H[0,0:2].dot(v) + H[0,2]) * z_inv
        u[1] = (H[1,0:2].dot(v) + H[1,2]) * z_inv
    elif l == 3:
        u = H @ v
    else:
        raise IndexError

    if is_nd:
        return u
    
    return u.tolist()

