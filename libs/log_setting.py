# -*- coding: utf-8 -*-
"""日志封装：同时输出到控制台和按天滚动的日志文件。"""
import os
import logging
from logging.handlers import TimedRotatingFileHandler


class CommonLog:
    def __init__(self, logger, logname="web-log"):
        self.logname = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "%s" % logname
        )
        self.logger = logger
        self.logger.setLevel(logging.DEBUG)
        self.logger.propagate = False
        self.formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s: %(message)s",
            "%Y-%m-%d %H:%M:%S",
        )

    def __console(self, level, message):
        fh = TimedRotatingFileHandler(
            self.logname, when="MIDNIGHT", interval=1, encoding="utf-8"
        )
        fh.suffix = "%Y-%m-%d.log"
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(self.formatter)
        self.logger.addHandler(fh)

        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG)
        ch.setFormatter(self.formatter)
        self.logger.addHandler(ch)

        if level == "error_":
            self.logger.error(message)
        else:
            getattr(self.logger, level)(message)

        self.logger.removeHandler(ch)
        self.logger.removeHandler(fh)
        fh.close()

    def debug(self, msg):
        self.__console("debug", msg)

    def info(self, msg):
        self.__console("info", msg)

    def warning(self, msg):
        self.__console("warning", msg)

    def error(self, msg):
        self.__console("error", msg)

    def error_(self, msg):
        self.__console("error_", msg)
