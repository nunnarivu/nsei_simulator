from dataset.nsrm.construct.base import DatasetConstructBase, DatasetWorld
from dataset.spg.construct.program_engine import Executor
from dataset.spg.construct.templates.program_lib import PROGRAM_LIB
from dataset.spg.configs import ParameterSettings as settings


class SPGDatasetWorld(DatasetWorld):
    def __init__(self, simulator_handle, configs):
        super().__init__(simulator_handle, configs)
    
    def _top_target_pos(self, position, dim):
        x, y, z = position
        dx, dy, dz = dim
        return (x, y, z + dz)
    
    def _right_target_pos(self, position, dim):
        x, y, z = position
        dx, dy, dz = dim
        return (x + dx + settings.SPATIAL_TARGET_MARGIN, y, z)

    def _front_target_pos(self, position, dim):
        x, y, z = position
        dx, dy, dz = dim
        return (x, y + dy + settings.SPATIAL_TARGET_MARGIN, z)
    
    def _left_target_pos(self, position, dim):
        x, y, z = position
        dx, dy, dz = dim
        return (x - dx - settings.SPATIAL_TARGET_MARGIN, y, z)
    
    def _back_target_pos(self, position, dim):
        x, y, z = position
        dx, dy, dz = dim
        return (x, y - dy - settings.SPATIAL_TARGET_MARGIN, z)

class SPGConstructBase(DatasetConstructBase):
    def __init__(self, simulator_handle, configs):
        super().__init__(simulator_handle, configs)
        self.program_engine = Executor(self, program_library=PROGRAM_LIB)
        self.dataset_world = SPGDatasetWorld(simulator_handle, configs)

    def check_action_compatibility(self, program, block_positions):
        raise NotImplementedError("This method should be implemented by subclasses.")

    
    
    
    def execute_program(self, program, dry_run = False):
        status, moved_positions, plan = self.program_engine.execute(program, dry_run)
        return status, moved_positions, plan