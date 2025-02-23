from envs.bullet.world import World
from envs.bullet.robot import get_robot_cls
import json
import pybullet as p
from envs.bullet.utils import check_config, config_to_bullet_params
import warnings
import time

class SimulatorInterface(object):
    def __init__(self, simulator_configs):
        self.config_file = simulator_configs
        if not check_config(simulator_configs):
            raise ValueError(f"Config file {simulator_configs} not found or in incorrect format")
        
        self._connect_to_client(simulator_configs)
        self.world = World(self.physics_client)
        
    
    def _connect_to_client(self, simulator_configs):
        '''
        Connect to the physics client with the given config file
        '''
        
        self.bullet_configs  = config_to_bullet_params(simulator_configs)
        self.physics_client = p.connect(self.bullet_configs["connection_mode"])
        
        if self.bullet_configs['gravity'] is not None:
            p.setGravity(self.bullet_configs['gravity'])
            self.gravity = self.bullet_configs['gravity']
        
        if self.bullet_configs['time_step'] is not None:
            p.setTimeStep(self.bullet_configs['time_step'])
            self.time_step = self.bullet_configs['time_step']
             
        if self.bullet_configs['real_time'] is not None:
            p.setRealTimeSimulation(self.bullet_configs['real_time'])
            self.real_time = self.bullet_configs['real_time']
        
        
    @property
    def robot(self):
        '''
        Returns the first robot in the world
        '''
        if len(self.world.robots) == 0:
            return None
        elif len(self.world.robots) > 1:
            warnings.warn("More than one robot in the world. Returning the first robot. Use world.robots attribute to access all robots")  
        
        return self.world.robots[0]
    
    
    def simulate_world(self, num_steps:int):
        '''
        Simulate the world for num_steps steps
        '''
        for _ in range(num_steps):
            p.stepSimulation()
            time.sleep(self.time_step)
    
    
    def disconnect(self):
        '''
        Disconnect from the physics client
        '''
        p.disconnect()
        self.physics_client = None
        self.world = None
        self.bullet_configs = None
        self.gravity = None
        self.time_step = None
        self.real_time = None
        
    
    def connect(self):
        '''
        Reconnect to the physics client and initialize the world
        '''
        self._connect_to_client(self.config_file)
        self.world = World(self.physics_client)
    
    
    
    
    
        
        
            
        
    
    