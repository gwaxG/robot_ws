import os
import importlib
import logging
import sys


class StreamToLogger(object):
    """
    Fake file-like stream object that redirects writes to a logger instance.
    """

    def __init__(self, logger, log_level=logging.INFO):
        self.logger = logger
        self.log_level = log_level
        self.linebuf = ''

    def write(self, buf):
        for line in buf.rstrip().splitlines():
            self.logger.log(self.log_level, line.rstrip())

    def flush(self):
        pass

    def close(self):
        pass


class BaseLogger(object):
    def __init__(self, fname, logger_name):
        # reload is necessary because rospy disables logging module
        importlib.reload(logging)
        direc, _ = os.path.split(__file__)
        log_file = os.path.join(direc, fname)
        print(log_file)
        logging.basicConfig(
            filename=log_file,
            filemode="w",
            level=logging.INFO,
            format='%(asctime)s - %(message)s'
        )
        # stdout_logger = logging.getLogger('STDOUT')
        # stderr_logger = logging.getLogger('STDERR')
        # sys.stdout = StreamToLogger(stdout_logger, logging.INFO)
        # sys.stderr = StreamToLogger(stderr_logger, logging.ERROR)
        self.logger = logging.getLogger(logger_name)

    def __del__(self):
        sys.stdout.close()
