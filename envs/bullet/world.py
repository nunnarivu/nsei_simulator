
import pybullet as p
import pybullet_data
import json

from envs.bullet.robot import PandaRobot


class Entity(object):
    def __init__(self, physics_client, id:int, name:str, pos:tuple, ori:tuple, model_path:str = None):
        self.physics_client = physics_client
        self.id = id
        self.name = name
        self.pos = pos
        self.ori = ori
        self.model_path = model_path
        
    def __str__(self):
        return json.dumps({"id": self.id, "name": self.name, "pos": self.pos, "ori": self.ori, "model_path": self.model_path})
    
    def remove(self):
        self.physics_client.removeBody(self.id)
    
    def update_pose(self, pos = None, ori = None):
        if pos is not None:
            self.pos = pos
        if ori is not None:
            self.ori = ori
        
        
class Object(Entity):
    def __init__(self, physics_client, id:int, name:str, pos:tuple, ori:tuple, model_path:str, obj_id:int):
        super().__init__(physics_client, id, name, pos, ori, model_path)
        self.obj_id = obj_id
        self.dim = physics_client.getVisualShapeData(self.id)[0][3]
        
        
    

class SpecialObject(Object):
    def __init__(self,physics_client, id:int, name:str, pos:tuple, ori:tuple, model_path:str, obj_id:int, tag:str):
        super().__init__(physics_client, id, name, pos, ori, model_path, obj_id)
        self.tag = tag
        self.attributes = {}
    
    def get_attribute(self, key:str):
        return self.attributes[key]
    
    def set_attribute(self, key:str, value):
        self.attributes[key] = value      
           



class World(object):
    def __init__(self, physics_client):
        self.physics_client = physics_client
        self.objects = []
        self.robots = []
        self.entities = []
        self.special_objects = []
    
    def _check_unique_name(self, name:str, obj_list:list):
        for obj in obj_list:
            if obj.name == name:
                return False
        return True
    
    
    def add_object(self, name:str, pos:tuple, ori:tuple, model_path:str):
        '''
        Add an object from the URDF taht needs to be manipulated by the robot. 
        It assigns an object ID and name to every objects added 
        '''
        id = self.physics_client.loadURDF(model_path, pos, ori)
        obj_id = len(self.objects)
        if not self._check_unique_name(name, self.objects):
            raise ValueError(f"Object with name {name} already exists")
        obj = Object(self.physics_client, id, name, pos, ori, model_path, obj_id)
        self.objects.append(obj)
        return obj
    
    
    def add_entity(self, name:str, pos:tuple, ori:tuple, model_path:str = None):
        '''
        Loads the given URDF file. 
        
        Desc: The urdf file can be plane, table or any other object. The object is loaded at the given position and orientation
        '''
        id = self.physics_client.loadURDF(model_path, pos, ori)
        if not self._check_unique_name(name, self.entities):
            raise ValueError(f"Entity with name {name} already exists")
        entity = Entity(self.physics_client, id, name, pos, ori, model_path)
        self.entities.append(entity)
        return entity

    def add_special_object(self, name:str, pos:tuple, ori:tuple, model_path:str, tag:str):
        id = self.physics_client.loadURDF(model_path, pos, ori)
        obj_id = len(self.special_objects)
        if not self._check_unique_name(name, self.special_objects):
            raise ValueError(f"Special Object with name {name} already exists")
        obj = SpecialObject(self.physics_client, id, name, pos, ori, model_path, obj_id, tag)
        self.special_objects.append(obj)
        return obj
    
    def add_robot(self, name:str, pos:tuple, ori:tuple, model_path:str, robot_cls):
        id = self.physics_client.loadURDF(model_path, pos, ori)
        robot = robot_cls(id, name, pos, ori, model_path)
        self.robots.append(robot)
        return robot
    
    
    def _search_by_name(self, name:str):
        for obj in self.objects:
            if obj.name == name:
                return obj, self.objects
        
        for obj in self.special_objects:
            if obj.name == name:
                return obj, self.special_objects
        
        for entity in self.entities:
            if entity.name == name:
                return entity, self.entities
        
        for robot in self.robots:
            if robot.name == name:
                return robot, self.robots
        return None, None
    
    
    def search_by_name(self, name:str):
        obj, obj_list = self._search_by_name(name)
        return obj
    
    
    def retrieve_type(self, name:str):
        obj, obj_list = self._search_by_name(name)
        if obj is not None:
            return type(obj)
        return None


    def remove_by_name(self, name:str):
        obj, obj_list = self._search_by_name(name)
        if obj is not None:
            obj.remove()
            obj_list.remove(obj)
    
    
    
    
    
        

