
import sys
import os
import pdb

# ----------------------------------------------------------------------------------------- Colours!

# See: http://misc.flogisoft.com/bash/tip_colors_and_formatting
class bcolors:
    """
    A set of constants that color terminal output.
    If no 'tty' is attached to the running process, then
    color constants are empty strings. Colors can be forced on 
    or off by setting the environment variable MULTIVIEW_COLORS.
    In this case, colors are on if MULTIVIEW_COLORS=1, and off,
    otherwise, irrespective of whether a 'tty' is attached.

    Example:

        print('{0}Red!{1}'.format(bcolors.RED, bcolors.ENDC))

    """
    
    has_tty = sys.stdout.isatty()

    if 'MULTIVIEW_COLORS' in os.environ:
        has_tty = os.environ['MULTIVIEW_COLORS'] == 1
    
    # Foreground Colors
    DEFAULT          = '\033[39m' if has_tty else ''
    BLACK            = '\033[30m' if has_tty else ''
    RED              = '\033[31m' if has_tty else ''
    GREEN            = '\033[32m' if has_tty else ''
    YELLOW           = '\033[33m' if has_tty else ''
    BLUE             = '\033[34m' if has_tty else ''
    MAGENTA          = '\033[35m' if has_tty else ''
    CYAN             = '\033[36m' if has_tty else ''
    LIGHT_GRAY       = '\033[37m' if has_tty else ''
    DARK_GRAY        = '\033[90m' if has_tty else ''
    LIGHT_RED        = '\033[91m' if has_tty else ''
    LIGHT_GREEN      = '\033[92m' if has_tty else ''
    LIGHT_YELLOW     = '\033[93m' if has_tty else ''
    LIGHT_BLUE       = '\033[94m' if has_tty else ''
    LIGHT_MAGENTA    = '\033[95m' if has_tty else ''
    LIGHT_CYAN       = '\033[96m' if has_tty else ''
    LIGHT_WHITE      = '\033[97m' if has_tty else ''

    # Background Colors
    BG_DEFAULT       = '\033[49m' if has_tty else ''
    BG_BLACK         = '\033[40m' if has_tty else ''
    BG_RED           = '\033[41m' if has_tty else ''
    BG_GREEN         = '\033[42m' if has_tty else ''
    BG_YELLOW        = '\033[43m' if has_tty else ''
    BG_BLUE          = '\033[44m' if has_tty else ''
    BG_MAGENTA       = '\033[45m' if has_tty else ''
    BG_CYAN          = '\033[46m' if has_tty else ''
    BG_LIGHT_GRAY    = '\033[47m' if has_tty else ''
    BG_DARK_GRAY     = '\033[100m' if has_tty else ''
    BG_LIGHT_RED     = '\033[101m' if has_tty else ''
    BG_LIGHT_GREEN   = '\033[102m' if has_tty else ''
    BG_LIGHT_YELLOW  = '\033[103m' if has_tty else ''
    BG_LIGHT_BLUE    = '\033[104m' if has_tty else ''
    BG_LIGHT_MAGENTA = '\033[105m' if has_tty else ''
    BG_LIGHT_CYAN    = '\033[106m' if has_tty else ''
    BG_LIGHT_WHITE   = '\033[107m' if has_tty else ''
    
    # Adjustments
    BRIGHT           = '\033[1m' if has_tty else ''
    DIM              = '\033[2m' if has_tty else ''
    UNDERLINED       = '\033[4m' if has_tty else ''
    BLINK            = '\033[5m' if has_tty else ''
    INVERTED         = '\033[7m' if has_tty else ''
    HIDDEN           = '\033[8m' if has_tty else ''    

    # Standard sequences
    INFO             = '\033[36m' if has_tty else ''
    WARN             = '\033[33m' if has_tty else ''
    ERROR            = '\033[91m' if has_tty else ''

    # Clear
    ENDC             = '\033[0m' if has_tty else ''

# ------------------------------------------------------------------------------ Print err/warn/info

def print_err(msg):
    """Print an error message with a colored "ERROR" prefix."""
    print(bcolors.ERROR + 'ERROR: ' + bcolors.ENDC + msg)

def print_warn(msg):
    """Print a warning message with a colored "WARN" prefix."""
    print(bcolors.WARN + 'WARN: ' + bcolors.ENDC + msg)

def print_info(msg):
    """Print an info message with a colored "INFO" prefix."""
    print(bcolors.INFO + 'INFO: ' + bcolors.ENDC + msg)

# ----------------------------------------------------------------------- Printing "banner" messages

class _Banner:

    def splitlines_with_wrap(text, w):
        lines = []
        for raw in text.splitlines():        
            if(len(raw) < w):
                lines.append(raw)
            else:
                while(len(raw) > 0):
                    lines.append(raw[0:w])
                    raw = raw[w:]            
        return lines
         
    def print_big(msg, w, header, skirt_style, banner_style):
        head_lines = _Banner.splitlines_with_wrap(header, w-4)
        print(skirt_style + ('-' * w) + bcolors.ENDC)
        for line in head_lines:
            off = int((w - 4 - len(line))/2.0)            
            padded = (' '*off) + line + (' '*(w - 4 - off - len(line)))
            assert(len(padded) == w - 4)
            print(skirt_style + '- ' + bcolors.ENDC + \
                  banner_style + padded + bcolors.ENDC + \
                  skirt_style + ' -' + bcolors.ENDC)
        _Banner.print_banner(msg, w, skirt_style)

    def print_banner(msg, w, skirt_style):
        lines = _Banner.splitlines_with_wrap(msg, w-4) 
        print(skirt_style + ('-' * w) + bcolors.ENDC)
        for line in lines:
            print(skirt_style + '- ' + bcolors.ENDC + line + \
                  (' ' * (w - 4 - len(line))) + \
                  skirt_style + ' -' + bcolors.ENDC)
        print(skirt_style + ('-' * w) + bcolors.ENDC)
        print("")

def print_big_banner(msg, heading, w=60,
                     skirt_style = bcolors.BRIGHT + bcolors.INFO,
                     banner_style = bcolors.INFO):
    """Prints `msg` in a colored (ASCII) box. `heading` is printed
    in a banner in the box. The box is `w` characters wide; defaults 
    to 60 characters.

    Args:
        
        msg:str          : The message to print.
        heading:str      : Message to print in the banner's header.
        w:int            : Width of the rendered banner.
        skirt_style:str  : Style (bash colors) for the banner's skirt.
        banner_style:str : Style for the banner's header text.

    """
    print("")
    _Banner.print_big(msg, w, heading, skirt_style, banner_style)
    print("")
    
def print_banner_warn(msg, heading='WARNING', w=60):
    """
    Prints `msg` in a banner with warning colors. The banner is `w` characters,
    and the bannder heading is `heading`.
    """
    print_big_banner(msg, heading, w, bcolors.BRIGHT + bcolors.WARN, bcolors.WARN)
        
def print_banner_err(msg, heading='ERROR', w=60):
    """
    Prints `msg` in a banner with error colors. The banner is `w` characters,
    and the bannder heading is `heading`.
    """
    print_big_banner(msg, heading, w,
                     bcolors.BRIGHT + bcolors.LIGHT_RED + bcolors.BG_LIGHT_RED,
                     bcolors.BG_LIGHT_CYAN + bcolors.BLINK + bcolors.BRIGHT + bcolors.RED)

def print_banner(msg, w=60):
    """
    Prints `msg` in a banner with info colors. The banner is `w` characters,
    and the bannder heading is `heading`.
    """
    print("")
    _Banner.print_banner(msg, w, bcolors.BRIGHT + bcolors.CYAN)
    print("")

