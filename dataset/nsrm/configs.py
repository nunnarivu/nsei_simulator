'''
REQUIRED_DATASET_CONFIGS:
    required_object_types(List): The objects that we want in the scene - a sublist of ['Cube','Tray','Lego','Dice']
    num_objexts_per_type(dict): keys = [num_+Obj[0].lower()+Obj[1:]+'s' for Obj in required_object_types]. If not given, the num of objects for each required_type  will be randomly choosen
    rotation(Bool): If True the objects will be rotated randomly which the scene gets initialized
    euler_orientation(3-tuple): By default orientation is same as that of the object in the urdf file. No rotation is applied. Provide a value if you want all the objects to be rotated by a same value.

'''





simulator_configs  = {
    "GUI_MODE": True,
    "resolution": [1024, 768], #Width, Height
    "record": None, #VideoPath, FPS
    "gravity": [0, 0, -9.81],
    "time_step": 1.0/240, #1/fps
    "real_time": False,
}


required_dataset_configs = {
    'MAX_OBJECTS': 4,
   'required_object_types' :['cube', "dice"], #Type: List, Options: 'cube', 'lego', 'dice', Default: ['cube', 'dice']
   # 'num_objects_per_type' :{'cube':3,'lego':2,'dice':1}, #If not given, num_objects will get randomly initialized
   'rotation': False,
   'num_instruction_per_scene':1,
   #euler_orientation = (0,0,0) 


}


additional_dataset_configs = {
   'instantiation_type' : 'random', #Type: str, options: ['random', 'default'], Default: 'default
   'complexity' : 'simple', #options: ['simple', 'complex', 'compound'], Default: Randomly choosen
   'relations' : ['left', 'right', 'behind', 'front'], #Use this to restrict relations. If this key is not there, the relational_concepts will be sampled from ["left", "right", "behind", "front"]
   'max_program_generation_atempts': 3000 #For each scene, the simulator will try to find a compatible program. This key restricts the number of such attempts. If all attempts failed, then the scene will get deleted.
}


class ParameterSettings(object):
    '''
    This class defines default parameter values for the Dataset and Simulator
    Currently the variables are defined as class variables. They can be changed to instance variables if needed.
    
    Note: The values are connected to each other. Changing one value may affect the other values.
    For examlpe, if panda position changes then the directions and the cameras_view may also needs to changed.
    '''
    TABLE_WORK_AREA = 0.8 #scale the work area. scale =1 means full table is used
    PANDA_POSITION = [0, 0.5, 0.5]
    TABLE_OFFSET = 0.65
    SPATIAL_TARGET_MARGIN = 0.03  #delta for the spatial target position like left, right etc
    COLOR = {'blue': (0, 0, 1, 1), 
            'green': (0, 1, 0, 1), 
            'red': (1, 0, 0, 1), 
            'yellow': (1, 1, 0, 1), 
            'white': (1, 1, 1, 1), 
            'magenta': (1, 0, 1, 1), 
            'cyan': (0, 1, 1, 1)
            }
    
    OBJECT_URDF_PATHS = {
        "plane": "plane.urdf",
        "table": "table/table.urdf",
        "cube": "cube_small.urdf",
        "tray": "tray/tray.urdf",
        "lego": "lego/lego.urdf",
        "blue_dice": "objects/dice/blue/Dice.urdf",
        "green_dice": "objects/dice/green/Dice.urdf",
        "red_dice": "objects/dice/red/Dice.urdf",
        "yellow_dice": "objects/dice/yellow/Dice.urdf",
        "white_dice": "objects/dice/white/Dice.urdf",
        "magenta_dice": "objects/dice/magenta/Dice.urdf",
        "cyan_dice": "objects/dice/cyan/Dice.urdf",
        }
    OBJECT_DIMENSIONS = {
        "cube": 0.05,
        "dice": 0.05,
        "lego": 0.0625
        }
    
    DIRECTIONS = {
        "top": [0., 0., 1.],
        "below": [0., 0., -1.],
        "left": [-1., 0., 0.],
        "right": [1., 0., 0.],
        "front": [0., -1., 0.],
        "back": [0., 1., 0.]        
        }
    
    CAMERA_VIEWS = {
        'front_0' : {
			'camera_eye_position': [0, 0, 0.5],
			'camera_target_position': [0, 0, 0],
			'camera_up_vector': [0, 1, 0],
			'fov': 60,
			'aspect': 1.33,
			'near_val': 0.1,
			'far_val': 100
            },
        'diag1_45' : {
            'camera_eye_position': [0, -0.5, 0.5],
            'camera_target_position': [0, 0, 0],
            'camera_up_vector': [0, 0, 1],
            'fov': 60,
            'aspect': 1.33,
            'near_val': 0.1,
            'far_val': 100
            },
        }
    ##END OF CLASS
    
