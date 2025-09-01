
import importlib


class Sensors(object):
    def __init__(self, physics_client):
        self.physics_client = physics_client
        self.sensor_list = []
        
    def __getstate__(self):
        state = self.__dict__.copy()
        # Store module by name instead of the object
        if "physics_client" in state:
            state["physics_client"] = state["physics_client"].__name__
        return state

    def __setstate__(self, state):
        # Re-import the module by name
        module_name = state["physics_client"]
        state["physics_client"] = importlib.import_module(module_name)
        self.__dict__.update(state)
        
    def add_sensor(self, sensor_type: str, sensor_params: dict, sensor_name = None):
        if sensor_type == 'rgbd':
            from envs.bullet.sensors.rgbd import RGBDSensor
            sensor = RGBDSensor(self.physics_client, sensor_params['camera_info'], sensor_params['image_info'])
            self.sensor_list.append((sensor, sensor_type, sensor_name))
        else:
            raise ValueError(f"Sensor type {sensor_type} not supported")

    def get_sensor(self, type, all = False, name = None):
        sensors = []
        for sensor, sensor_type, sensor_name in self.sensor_list:
            if sensor_type == type and (name is None or sensor_name == name):
                sensors.append(sensor)
        return sensors if all else sensors[0] if sensors else None
    