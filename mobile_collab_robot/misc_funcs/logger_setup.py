import logging
import sys

from mobile_collab_robot.defs import project_root


def setup_logger(name, logging_level=logging.INFO):
    log_file = project_root() / "mobile_collab_robot.log"
    formatter = logging.Formatter("%(asctime)s %(name)s %(levelname)s: %(message)s")

    logger = logging.getLogger(name)
    logger.setLevel(logging_level)
    if logger.hasHandlers():
        logger.handlers.clear()

    # log_handler_stdout = logging.StreamHandler(sys.stdout)
    # log_handler_stdout.setFormatter(formatter)
    # logger.addHandler(log_handler_stdout)

    log_handler_file = logging.FileHandler(log_file)
    log_handler_file.setFormatter(formatter)
    logger.addHandler(log_handler_file)

    return logger
