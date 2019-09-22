import inspect, logging
import datetime

def log(message):
    logger = logging.getLogger('main')
    logger.setLevel(logging.DEBUG)
    func = inspect.currentframe().f_back.f_code
    print("%s [%s:%i - %s()] %s" % (
        datetime.datetime.now(),
        func.co_filename,
        func.co_firstlineno,
        func.co_name,
        message
    ))
