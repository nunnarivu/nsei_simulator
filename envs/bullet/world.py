
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

    def change_color(self, color:tuple):
        self.physics_client.changeVisualShape(self.id, -1, rgbaColor=color)
        
    def get_body_position(self):
        return self.physics_client.getBasePositionAndOrientation(self.id)[0]
    
    def get_body_orientation(self):
        return self.physics_client.getBasePositionAndOrientation(self.id)[1]
    
    def get_body_bounding_box(self):
        return self.physics_client.getAABB(self.id)
    
    def get_body_dimensions(self):
        vis_data = self.physics_client.getVisualShapeData(self.id)
        return [vis_data[i][3] for i in range(len(vis_data))]
    
    def is_occupying(self, position):
        for link_id in range(-1, self.physics_client.getNumJoints(self.id)):
            aabb = self.physics_client.getAABB(self.id, link_id)
            if (aabb[0][0] <= position[0] <= aabb[1][0] and
                aabb[0][1] <= position[1] <= aabb[1][1] and
                aabb[0][2] <= position[2] <= aabb[1][2]):
                return True
        return False

class Object(Entity):
    def __init__(self, physics_client, id:int, name:str, pos:tuple, ori:tuple, model_path:str, obj_id:int):
        super().__init__(physics_client, id, name, pos, ori, model_path)
        self.obj_id = obj_id
        self.dim = physics_client.getVisualShapeData(self.id)[0][3]
    
    def __str__(self):
        base_str = json.dumps({ "id": self.id, "name": self.name, "pos": self.pos, "ori": self.ori,"model_path": self.model_path, "obj_id": self.obj_id, })
        return base_str
    
    


class SpecialObject(Object):
    def __init__(self,physics_client, id:int, name:str, pos:tuple, ori:tuple, model_path:str, obj_id:int, tag:str):
        super().__init__(physics_client, id, name, pos, ori, model_path, obj_id)
        self.tag = tag
        self.attributes = {}
    
    def get_attribute(self, key:str):
        return self.attributes[key]
    
    def set_attribute(self, key:str, value):
        self.attributes[key] = value 
        
    def __str__(self):
        base_str = json.dumps({ "id": self.id, "name": self.name, "pos": self.pos, "ori": self.ori,"model_path": self.model_path, "obj_id": self.obj_id, "tag": self.tag, })
        return base_str

class WorldUtils(object):
    def __init__(self, physics_client):
        self.physics_client = physics_client
        self.objects = []
        self.robots = []
        self.entities = []
        self.special_objects = []
    
    @property
    def class2type(self):
        return {
            "Entity": "entity",
            "Object": "object",
            "SpecialObject": "special_object",
            "Robot": "robot"
        }

    def _check_unique_name(self, name:str, obj_list:list):
        for obj in obj_list:
            if obj.name == name:
                return False
        return True
    
    def _add_object(self, name:str, pos:tuple, ori:tuple, model_path:str):
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
    
    def _add_entity(self, name:str, pos:tuple, ori:tuple, model_path:str = None):
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

    def _add_special_object(self, name:str, pos:tuple, ori:tuple, model_path:str, tag:str):
        id = self.physics_client.loadURDF(model_path, pos, ori)
        obj_id = len(self.special_objects)
        if not self._check_unique_name(name, self.special_objects):
            raise ValueError(f"Special Object with name {name} already exists")
        obj = SpecialObject(self.physics_client, id, name, pos, ori, model_path, obj_id, tag)
        self.special_objects.append(obj)
        return obj
    
    def _add_robot(self, name:str, pos:tuple, ori:tuple, model_path:str, robot_cls):
        id = self.physics_client.loadURDF(model_path, pos, ori)
        robot = robot_cls(id, name, pos, ori, model_path)
        self.robots.append(robot)
        return robot
    
    def _search_by_name(self, name:str) -> tuple[Entity, list]:        
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


class World(WorldUtils):
    def __init__(self, physics_client):
        super().__init__(physics_client)
        self.checkpoint_cache = {} 
        self.state_cache = {} 
        self.state_transition_history = []

    def add(self, name:str, pos:tuple, ori:tuple, obj_type:str = 'object', **kwargs):
        '''
        Add an object to the world. 
        
        Desc: The object can be a robot, object or entity. The object is loaded at the given position and orientation
        '''
        model_path = kwargs.get('model_path', None)
        if model_path is None:
            raise ValueError("Model path must be provided. Automatic object search is not implemented!") 
        if obj_type == 'object':
            return self._add_object(name, pos, ori, model_path)
        elif obj_type == 'entity':
            return self._add_entity(name, pos, ori, model_path)
        elif obj_type == 'special_object':
            if kwargs.get('tag') is None:
                raise ValueError("Tag must be provided for special objects")
            return self._add_special_object(name, pos, ori, model_path, kwargs.get('tag'))
        elif obj_type == 'robot':
            if kwargs.get('robot_cls') is None:
                raise ValueError("Robot class must be provided for robots")
            return self._add_robot(name, pos, ori, model_path, kwargs.get('robot_cls'))
        else:
            raise ValueError(f"Object type {obj_type} not supported")

    def get_body_dimensions(self, name:str):
        obj, obj_list = self._search_by_name(name)
        if obj is not None:
            return obj.get_body_dimensions()
        return None

    def get_body_position(self, name:str):
        obj, obj_list = self._search_by_name(name)
        if obj is not None:
            return obj.get_body_position()
        return None

    def find(self, name:str = None, id:int = None, type:str = None):
        if name is not None:
            obj, obj_list = self._search_by_name(name)
            return obj
        if id is not None:
            for obj in self.objects + self.special_objects + self.entities + self.robots:
                if obj.id == id:
                    return obj
        if type is not None:
            obj_list = eval(f'self.{type}s')
            return obj_list
        return None

    def retrieve_type(self, name:str) -> str:
        obj, obj_list = self._search_by_name(name)
        if obj is not None:
            return self.class2type[type(obj).__name__]
        return None

    def remove(self, name:str) -> None:
        obj, obj_list = self._search_by_name(name)
        if obj is not None:
            obj.remove()
            obj_list.remove(obj)

    def remove_all(self, types:list = None) -> None:
        if types is None:
            types = ['object', 'entity', 'special_object', 'robot']

        for obj_type in types:
            assert obj_type in ['object', 'entity', 'special_object', 'robot'], f"Object type {obj_type} not supported"
            obj_list = eval(f'self.{obj_type}s')
            for obj in obj_list:
                obj.remove()
                obj_list.remove(obj)
                
    
    def cache_state(self, tag = None) -> None:
        if tag is None:
            tag = "state_"+str(len(self.state_cache))
        state = {}
        for obj in self.objects + self.special_objects + self.entities + self.robots:
            pos, ori = p.getBasePositionAndOrientation(obj.id)
            linear_vel, angular_vel = p.getBaseVelocity(obj.id)
            state[obj.name] = {
                "pos": pos,
                "ori": ori,
                "linear_vel": linear_vel,
                "angular_vel": angular_vel,
                "class_type": type(obj).__name__,
                "type" : self.class2type[type(obj).__name__],
                "info": obj.__str__(),
            }
        self.state_cache[tag] = state
    
    def is_position_colliding(self, position:list, obj_name:str = None, obj_type:str = None):
        objects_to_check = []
        if obj_name is not None:
            obj = self.search_by_name(obj_name)
            if obj is not None:
                objects_to_check.append(obj)
        elif obj_type is not None:
            objects_to_check = eval(f'self.{obj_type}s')
        else:
            objects_to_check = self.objects + self.special_objects + self.entities + self.robots

        for obj in objects_to_check:
            if obj.is_occupying(position=position):
                return True
        return False

    def check_collision(self, obj_list):
        for i in range(len(obj_list)):
            for j in range(i+1, len(obj_list)):
                contacts = p.getContactPoints(bodyA=obj_list[i].id, bodyB=obj_list[j].id)
                if len(contacts) > 0:
                    return True
        return False