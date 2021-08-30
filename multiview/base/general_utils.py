
import getpass
import os

def static_vars(**kwargs):
    """Decorator for adding static variables to a function.
    
    Example:

        @static_vars(counter=0)
        def some_function():
            # Static variable keeps value between calls
            some_function.counter += 1
            return some_function.counter
    """
    def decorate(func):
        for k in kwargs:
            setattr(func, k, kwargs[k])
        return func
    return decorate


def get_user():
    """
    Get's the current user's username. `getpass` throws an
    exception if the environment isn't set up correctly --
    can happen in a docker container -- so this method falls
    back to the `uid` of the currently executing process.
    """
    ret = None
    try:
        ret = getpass.getuser()
    except:
        pass

    if ret is None: ret = os.getenv('USER')
    if ret is None: ret = os.getenv('USERNAME')        
    if ret is None: ret = '{}'.format(os.getuid())
        
    return ret
        
