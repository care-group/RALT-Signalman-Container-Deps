import yaml
from yaml import FullLoader

from log import Log

sensors = {}
sensor_labels = []

class LoadYAML():
    def __init__(self):
        self.id = 'load_yaml'

        self.logger = Log(self.id)

        self.logger.log_great('Ready.')

    def load_file(self, file):
        path = "/home/sandbox/shared/input/" + file
        stream = open(path, 'r')
        dictionary = yaml.load(stream, Loader=FullLoader)

        if not bool(dictionary):
            self.logger.log_warn('No content in YAML file.')
            exit()

        try:
            for key, value in dictionary.items():
                for key, contents in value.items():
                    sensors[key] = contents
                    sensor_labels.append(key)
        except:
            self.logger.log_warn('Invalid sensor configuration in YAML file. Please check syntax.')
            exit()

        for key, value in sensors.items():
            sensors[key]['semantic_state'] = 'null'
            sensors[key]['raw_state'] = 'null'

        return sensors, sensor_labels