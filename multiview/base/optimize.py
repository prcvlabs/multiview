
import math
import numpy as np
import pdb
import functools
import random
from . import colors

import scipy

# ------------------------------------------------------------------------------ Optimization Result

class OptResult:
    """Optimization result returned from the method `nelder_mead_with_restarts`."""
    def __init__(self):
        self.X = None                   # Resulting optimization parameters
        self.init_err = float('inf')    # Initial value of cost funciton
        self.convex_err = float('inf')  # Value of cost function after 0 restarts
        self.err = float('inf')         # Final value of cost function
        self.n_evals = 0                # Number of times cost-fun was evaluated
        self.n_restarts = 0             # Number of restarts

# ------------------------------------------------------------------------ Nelder-Mead with Restarts

def nelder_mead_with_restarts(cost_fun, X0, options={}):
    """
    Non-convex optimization method suitable for some non-convex problems. Uses 
    Nelder-Mead to perform convex optimization step, performing a variable number
    of restarts. The parameters (`X0`) are perturbed between restarts by a variable
    amount that starts small, and grows if no progress is being made. The 
    optional parameters `stdev` (a numpy.array of dimension `X0`) can be used to
    scale the amount that parameters are perturbed.

    Args:

        cost_fun:function : Objective function to optimize. Must take flat array of 
                            floating point numbers, e.g., cost_fun(X0)
        X0:np.array(N)    : An array of parameters that initializes estimation.
        options:dict      : Options passed to underlying scipy.minize, 
                            using method='nelder-mead'. Usuable options are specified
                            below.

    Options:

        // -- These options operate within convex optimize (nelder-mead) step -- //
        xtol:float        : Absolute differince in parameters (X) for termination.
        ftol:float        : Absolute difference in cost_fun for convergence to stop.
        maxfev:int        : Maximum number of cost_fun evaluations before termination.
                            Defaults to len(X0)*200
        maxiter:int       : Maximum number of iterations before optimization stops.
        disp:bool         : True => print convergence messages.

        // -- These options operate between restarts -- //
        good_enough:float : No restarts if cost function is below this value, unless
                            the minimum number of restarts hasn't been met. Default is 1e-9.
        max_evals:int     : Maximum times to call cost_fun before giving up. If 0, 
                            then it is set to len(X0) * 1000. If negative, then there
                            is no maximum. Default 0.
        min_restarts:int  : Minimum number of attempts to improve result. Default 0.   
        max_restarts:int  : Maximum number of attempts to improve result. If negative
                            then there is no maximum. If 0, then will _never_ restart.
                            Default -1.
        stdev:np.array(N) : Standard deviations used to guide perturbing parameters
                            between restarts. If no standard deviations are specified then 
                            parameters are perturbed by percentages.
        feedback:bool     : True => print feedback (at restart level, not convergence)
    """

    # Extract restart options
    good_enough = options['good_enough'] if 'good_enough' in options else 0.0
    max_evals = options['max_evals'] if 'max_evals' in options else 0
    min_restarts = options['min_restarts'] if 'min_restarts' in options else 0
    max_restarts = options['max_restarts'] if 'max_restarts' in options else -1
    stddev = options['stdev'] if 'stdev' in options else None
    feedback = options['feedback'] if 'feedback' in options else False
    
    if not stddev is None:
        print_banner_warn('Specifying stdev has not been tested')
    
    # Extract nelder-mead options
    nelder_mead_options={}
    if 'ftol' in options: nelder_mead_options['ftol'] = options['ftol']
    if 'xtol' in options: nelder_mead_options['xtol'] = options['xtol']
    if 'maxfev' in options: nelder_mead_options['maxfev'] = options['maxfev']
    if 'maxiter' in options: nelder_mead_options['maxiter'] = options['maxiter']
    if 'disp' in options: nelder_mead_options['disp'] = options['disp']
    
    # Wrap the cost funciton
    opt_result = OptResult()
    opt_result.X = np.copy(X0)
    opt_result.init_err = cost_fun(X0)
    opt_result.err = opt_result.init_err
    def xopt(X):
        nonlocal opt_result

        # Are we done with evaluating?
        opt_result.n_evals += 1
        if opt_result.n_evals > max_evals:
            raise RuntimeError

        # Keep track of the best evaluation so far
        err = cost_fun(X)
        if err < opt_result.err:
            opt_result.X = np.copy(X)
            opt_result.err = err
            
        return err
    
    # Run Nelder-Mead once with parameters 'X'
    def run_convex_step(X):
        initial_X = np.copy(X)
        err = float('inf')        
        try:
            res = scipy.optimize.minimize(xopt, X, method='nelder-mead',options=nelder_mead_options)
            if res.success:
                err = res.fun
        except:
            pass

        if feedback:
            init_err = cost_fun(initial_X)
            print('#{0:< 10d} {1:10.4f} => {2:10.4f} ({3:10.4f} => {3:10.4f})'
                  ''.format(opt_result.n_evals, init_err, err, opt_result.err))

    # Always run optimization once
    run_convex_step(X0)
    opt_result.convex_err = opt_result.err
    
    # Never restart if max_restarts == 0
    if max_restarts == 0:
        if feedback:
            print("-" * 60)
        return opt_result

    # Perturb functions
    def perturb_stddev(X, n_restarts_no_progress):
        # perturb scale: 8 restarts gives 1 standard deviation
        ps = (n_restarts_no_progress % 32 + 1) * 0.125
        return np.array([x + (random.random()-0.5)*ss*ps for x, ss in zip(X, stddev)], X0.dtype)

    def perturb_percentage(X, n_restarts_no_progress):
        perturb_scale = (n_restarts_no_progress % 32 + 1) * 0.025
        ss = random.random() * perturb_scale # Scale all random numbers by this amount
        return np.array([(1.0 + ((random.random() - 0.5) * ss)) * x for x in X], X0.dtype)
    
    # Manage the restarts
    n_restarts_no_progress = 0
    while opt_result.n_evals < max_evals:
        # If max-restarts < 0, then we never limit the number of restarts
        if max_restarts > 0 and opt_result.n_restarts < max_restarts:
            break
        
        # Additional stop condition: stop if err is good-enough, and we
        # haven't yet reached the minimum number of restarts
        if opt_result.err <= good_enough and opt_result.n_restarts >= min_restarts:
            break

        # Perturb, depending on whether or not 'stdev' was specified in the parameters
        X = perturb_percentage(np.copy(opt_result.X), n_restarts_no_progress) \
            if stddev is None else perturb_stddev(np.copy(opt_result.X), n_restarts_no_progress)
                
        # Restart, keeping track of whether we made progress
        opt_result.n_restarts += 1
        last_err = opt_result.err
        run_convex_step(X)
        if last_err <= opt_result.err: # Perturb didn't help
            n_restarts_no_progress += 1
        else:
            n_restarts_no_progress = 0

    if feedback:
        colors.print_big_banner('Initial error = {0}\n'
                                'Convex error  = {1}\n'
                                'Final error   = {2}\n'
                                'n-evals       = {3}\n'
                                'n-restarts    = {4}\n'
                                ''.format(opt_result.init_err,
                                          opt_result.convex_err,
                                          opt_result.err,
                                          opt_result.n_evals,
                                          opt_result.n_restarts),
                                'OPTIMIZATION RESULT')
    return opt_result
    
