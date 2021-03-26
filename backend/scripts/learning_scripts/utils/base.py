#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import os
import argparse
import datetime


class Base:
    """
    This class bears all logic to lighten learning scripts.
    1. Fake file-like stream object that redirects writes to a logger instance.
    2. Input parsing.
    3. Node creation.
    """

    def __init__(self):
        # parse input
        parser = argparse.ArgumentParser()
        parser.add_argument('-p', type=int, default=11311, help='ROS MASTER URI port')
        args = parser.parse_args()
        port = args.p

        # create log file
        file_name = os.path.join(os.path.split(__file__)[0], "log_"+str(port))
        self.f = open(file_name, "w")

        # save original streams
        self.save_stdout = sys.stdout
        self.save_stderr = sys.stderr

        # redirect streams
        sys.stdout = self
        sys.stderr = self

        # lambda function to log with date
        self.log = lambda s: print(datetime.datetime.now(), s)

        # ROS-related part
        os.environ["ROS_MASTER_URI"] = "http://localhost:" + str(port)
        rospy.init_node("example")
        self.rospy = rospy

    def write(self, buf):
        self.f.write(buf)
        self.f.flush()

    def flush(self):
        self.f.flush()

    def close(self):
        self.f.close()
        sys.stdout = self.save_stdout
        sys.stderr = self.save_stderr

    def __del__(self):
        self.close()
