
import numpy as np
from dataset.nsrm.configs import ParameterSettings as settings






class DatasetWorld(object):
    def __init__(self, simulator_handle=None, configs=None):
        self.simulator_handle = simulator_handle
        self.configs = configs

    def reset_world(self, ):
        self.simulator_handle.reset()
        plane = self.simulator_handle.world.add(name = 'plane', pos=(0, 0, 0), ori=(0, 0, 0, 1), obj_type='entity', model_path=settings.OBJECT_URDF_PATHS['plane'])
        table = self.simulator_handle.world.add(name = 'table', pos=(0, 0, 0), ori=(0, 0, 0, 1), obj_type='entity', model_path=settings.OBJECT_URDF_PATHS['table'])
        table.change_color([0.48, 0.435, 0.2, 1])
        self.simulator_handle.step_simulation(num_steps=100)
        
        
    def relative_target_pos(self, relation):
        pass
    
    def in_camera_view(self, pos):
        return self.simulator_handle.sensors.get_sensor('rgbd_1').is_inside_camera_view(pos)

    def get_random_table_position(self, ):
        table_dim  = self.simulator_handle.world.get_body_dimensions('table')[0]
        table_pos = self.simulator_handle.world.get_body_position('table')
        x = np.random.uniform(table_pos[0] - (table_dim[0] / 2)*settings.TABLE_WORK_AREA, table_pos[0] + (table_dim[0] / 2)*settings.TABLE_WORK_AREA)
        y = np.random.uniform(table_pos[1] - (table_dim[1] / 2)*settings.TABLE_WORK_AREA, table_pos[1] + (table_dim[1] / 2)*settings.TABLE_WORK_AREA)
        z = table_pos[2] + settings.TABLE_OFFSET + table_dim[2] / 2  # Place above the table
        return (x, y, z)

    def check_collision(self, ):
        obj_list = self.simulator_handle.world.find(type='object')
        if self.simulator_handle.world.check_collision(obj_list):
            return True
        return False

    def is_position_clear(self, pos):
        return not self.simulator_handle.world.is_position_colliding(pos, obj_type = 'object')

    def get_block_positions(self, num_blocks, ensure_visibility = False, ):
        if ensure_visibility:
            raise NotImplementedError
        block_positions =  None
        for i in range(1000):
            temp_block_positions = [self.get_random_table_position() for _ in range(num_blocks)]
            valid = all(self.is_position_clear(pos) for pos in temp_block_positions)
        
            if valid:
                if ensure_visibility and (not all(self.simulator_handle.is_inside_camera_view(pos) for pos in temp_block_positions)):
                    continue
                block_positions = temp_block_positions
                break
        return block_positions

    def _noise(self, ):
        pass
    
    def _inside_target_pos(self, ):
        pass
    
    def _top_target_pos(self, ):
        pass
    
    def _left_target_pos(self, ):
        pass
    
    def _right_target_pos(self, ):
        pass
    
    def _back_target_pos(self, ):
        pass
    
    def _front_target_pos(self, ):
        pass
    
    



class DatasetConstructBase(object):
    def __init__(self, simulator_handle, configs):
        self.dataset_world = DatasetWorld(simulator_handle, configs)
    
    @property
    def simulator_handle(self):
        return self.dataset_world.simulator_handle

    @property
    def configs(self):
        return self.dataset_world.configs

    def hide_robot_body(self, **kwargs):
        pass
    
    def  show_robot_body(self, ):
        pass
    
    def save_instance(self, ):
        pass
    
    def get_scene_info(self, ):
        pass
    
    def cache_current_position_info(self, ):
        pass
    
    def apply_program(self, ):
        pass
    
    def move_object(self, ):
        pass
    
    def check_action_compatiblity(self, ):
        pass
    
    
    
    