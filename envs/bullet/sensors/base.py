

class Sensors(object):
    def __init__(self, physics_client):
        self.physics_client = physics_client
        self.sensor_list = []
        
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
    