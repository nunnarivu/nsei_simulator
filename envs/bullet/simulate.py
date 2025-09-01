from envs.bullet.world import World
from envs.bullet.sensors import Sensors
# from envs.bullet.robot import get_robot_cls
import json, pickle
import os
import pybullet as p
import pybullet_data
from envs.bullet.utils import check_config, config_to_bullet_params
import warnings
import time

class SimulatorInterface(object):
    def __init__(self, simulator_configs):
        self.config_file = simulator_configs
        if not check_config(simulator_configs):
            raise ValueError(f"Config file {simulator_configs} not found or in incorrect format")
        
        self._connect_to_client(simulator_configs)
        self.checkpoint_cache = {}
        self.local_assets_path = os.path.join(os.path.dirname(__file__), "assets")

    def _connect_to_client(self, simulator_configs):
        '''
        Connect to the physics client with the given config file
        '''
        
        self.bullet_configs  = config_to_bullet_params(simulator_configs)
        connection_options = f"--width={self.bullet_configs['resolution'][0]} --height={self.bullet_configs['resolution'][1]}"
        if self.bullet_configs['record'] is not None:
            connection_options += f"--minGraphicsUpdateTimeMs=0 --mp4={self.bullet_configs['record'][0]} --fps={self.bullet_configs['record'][1]}"
        p.connect(self.bullet_configs["connection_mode"], options = connection_options)
        
        self.physics_client = p
        self.reset()
    
    def reset(self):
        self.physics_client.resetSimulation()
        
        if self.bullet_configs['gravity'] is not None:
            self.physics_client.setGravity(*self.bullet_configs['gravity'])
            self.gravity = self.bullet_configs['gravity']
        
        if self.bullet_configs['time_step'] is not None:
            self.physics_client.setTimeStep(self.bullet_configs['time_step'])
            self.time_step = self.bullet_configs['time_step']
            
        if self.bullet_configs['real_time'] is not None:
            self.physics_client.setRealTimeSimulation(self.bullet_configs['real_time'])
            self.real_time = self.bullet_configs['real_time']

        self.physics_client.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        self.step_simulation(10)

        self.world = World(self.physics_client)
        self.sensors = Sensors(self.physics_client)

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


    def step_simulation(self, num_steps:int):
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
        

    def save_checkpoint(self, tag:str = None, dump_to_file:bool = False, root_folder:str = None):
        state_id = self.physics_client.saveState()
        if tag is None:
            tag = "state_"+str(state_id)
        world_pickle = pickle.dumps(self.world)
        sensor_pickle = pickle.dumps(self.sensors)
        
        checkpoint = {"state_id": state_id, "world": world_pickle, "sensors": sensor_pickle}
        self.checkpoint_cache[tag]  = checkpoint
        
        if dump_to_file:
            if root_folder is None:
                root_folder = "checkpoints"
            os.makedirs(root_folder, exist_ok=True)
            path = os.path.join(root_folder, tag)
            self.physics_client.saveBullet(path + ".bullet")
            with open(path + ".pkl", "wb") as f:
                pickle.dump(self.checkpoint_cache[tag], f)
        return (tag, checkpoint)

    def load_checkpoint(self, tag:str = None, from_file:str = None, checkpoint:dict = None):          
        if from_file is not None:
            checkpoint = pickle.load(open(from_file, "rb"))
        
        if tag is not None:
            checkpoint = self.checkpoint_cache.get(tag)
            if checkpoint is None:
                raise ValueError(f"Checkpoint with tag {tag} not found")
        
        elif tag is None and checkpoint is None:
            warnings.warn("No tag/file/checkpoint_dict provided. Trying to load the last saved checkpoint.")
            assert len(self.checkpoint_cache) > 0, "No checkpoints available to load"
            tag = list(self.checkpoint_cache.keys())[-1]
            checkpoint = self.checkpoint_cache[tag]
        
        self.world = pickle.loads(checkpoint["world"])
        self.sensors = pickle.loads(checkpoint["sensors"])
        
        try:
            self.physics_client.restoreState(checkpoint["state_id"])
        except:
            warnings.warn("Failed to restore physics client state. The world may not be in the expected state. Trying to use bullet file if available")
            if from_file is not None:
                bullet_file = from_file.replace(".pkl", ".bullet")
                self.physics_client.restoreState(fileName = bullet_file)