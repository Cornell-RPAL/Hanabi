import inspect, logging
import datetime
from sys import argv

def log(*message):
    logger = logging.getLogger('main')
    logger.setLevel(logging.DEBUG)
    func = inspect.currentframe().f_back.f_code
    msg = "%s [%s:%i - %s()] %s" % (
        datetime.datetime.now(),
        func.co_filename,
        func.co_firstlineno,
        func.co_name,
        remove_parentheses(message)
    )
    if "-sv" in argv:
        logging.debug(msg)
    else:
        print(msg)

def remove_parentheses(args):
    out = ""
    for i in args:
        out += str(i)
    return out
