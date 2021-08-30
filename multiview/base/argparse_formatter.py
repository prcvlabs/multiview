
import argparse

class NullTextHelpFormatter(argparse.RawDescriptionHelpFormatter):
    """Text formatter for argparse help message that 
    (1) Doesn't mangle white space in the description, and
    (2) Doesn't print "helpful" optional and postional argument secionts.

    When using this text formatter, put all usage information in 
    the description field (of the argument parser), and format it
    however you want. Done."""
    def add_argument(self, action):
        pass

    
