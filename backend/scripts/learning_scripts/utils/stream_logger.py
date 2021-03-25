import sys
import os


class StreamLogger:
    """
    Fake file-like stream object that redirects writes to a logger instance.
    """

    def __init__(self, pid):
        file_name = os.path.join(os.path.split(__file__)[0], "log_"+str(pid))
        self.f = open(file_name, "w")
        self.save_stdout = sys.stdout
        self.save_stderr = sys.stderr
        sys.stdout = self
        sys.stderr = self

    def write(self, buf):
        self.f.write(buf)
        self.f.flush()

    def flush(self):
        self.f.flush()

    def close(self):
        self.f.close()
        sys.stdout = self.save_stdout
        sys.stderr = self.save_stderr