import requests

from log import Log
from time import sleep

class OpenHABHelper():
    def __init__(self, url, sensor_labels):
        self.id = 'openhab_helper'

        self.logger = Log(self.id)

        self.url = url
        self.sensor_labels = sensor_labels

        self.logger.log_great('Ready.')

    def update(self):
        try:
            response = requests.get(self.url)
        except:
            self.logger.log_warn('Unable to connect to OpenHAB endpoint. Check OPENHAB_URL. Retrying...')
            sleep(5)
            self.update()

        if response.status_code != 200:
            self.logger.log_warn('No response from OpenHAB!')
            return None

        items = response.json()
        
        return self.process(items)

    def process(self, items):
        relevant_items = {}

        for item in items:
            if item['label'] in self.sensor_labels:
                relevant_items[item['label']] = item

        return relevant_items
