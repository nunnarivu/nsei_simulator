
import os
import json
from envs.bullet.robot import PandaRobot
import pybullet as p

def check_config(config):
      
    if "GUI_MODE" in config:
        assert type(config["GUI_MODE"]) == bool, "GUI_MODE should be a boolean. True for GUI, False for headless"
    if "gravity" in config:
        assert type(config["gravity"]) == list and len(config["gravity"]) == 3, "gravity should be a list of 3 floats"
    if "time_step" in config:
        assert type(config["time_step"]) == float, "time_step should be a float"
    if "real_time" in config:
        assert type(config["real_time"]) == bool, "real_time should be a boolean.  If you enable the real-time simulation, you don't need to call 'stepSimulation'"
    return True

def config_to_bullet_params(config):
    bullet_configs = {}
    bullet_configs["connection_mode"] = p.GUI if config["GUI_MODE"] else p.DIRECT
    bullet_configs["gravity"] = config["gravity"] if "gravity" in config else None
    bullet_configs["time_step"] = config["time_step"] if "time_step" in config else None
    bullet_configs["real_time"] = config["real_time"] if "real_time" in config else None
    bullet_configs["resolution"] = config["resolution"] if "resolution" in config else [1024, 768]
    bullet_configs["record"] = config["record"] if "record" in config else None
    return bullet_configs


def get_robot_cls(robot_name):
    if robot_name == "panda":
        return PandaRobot
    else:
        raise ValueError(f"Robot {robot_name} not found")
    
