#! /usr/bin/env python
import rospy
import threading
from time import sleep, strftime, time
from log import Log
from load_yaml import LoadYAML
from openhab_helper import OpenHABHelper
from csv_tools import CSVTools
from detect_events import DetectEvents

from std_msgs.msg import String

SENSORS_CONFIG = "sensors.yaml"
PERIODICITY = 1.0
OPENHAB_URL = "https://caregrouphwu%40icloud.com:G00drobot6@home.myopenhab.org/rest/items"
# OPENHAB_URL = "http://192.168.1.130:8080/rest/items"

class Main():
    def __init__(self, pub):
        self.id = 'main'

        date_time = strftime("%Y%m%d-%H%M%S")

        self.pub = pub

        self.logger = Log(self.id)
        self.logger.startup_msg()

        # * * * CONFIGURATION AREA * * * 

        # set to True for real-time mode (1 second/sample)
        self.real_time = True

        # * * * * * * * * * * * * * * *

        # Load YAML
        self.load_yaml = LoadYAML()
        self.sensors, self.sensor_labels = self.load_yaml.load_file(SENSORS_CONFIG)

        # Set up CSV files
        self.csv_tools = CSVTools()
        self.csv_tools.create_event_file()

        # Set up OpenHAB helper
        self.openhab_helper = OpenHABHelper(OPENHAB_URL, self.sensor_labels)

        # Set up event detector
        self.detect_events = DetectEvents(self.sensors, self.sensor_labels)

        self.step = 0

        self.logger.log_great('Ready.')

    def loop(self):
        while(True):
            if self.real_time:
                start_time = time()

            self.current_state = self.openhab_helper.update()
            
            if self.step == 0:
                self.detect_events.init_semantic_state(self.current_state)

            if self.step > 0:
                events = self.detect_events.step(self.current_state, self.previous_state, self.step)
                if len(events) > 0:
                    self.csv_tools.write_events(events)
                    message = str(events)
                    if not rospy.is_shutdown():
                        self.pub.publish(message)

            if self.real_time:
                end_time = time()
                time_taken = end_time - start_time
                delay_time = PERIODICITY - time_taken

                if delay_time >= 0.0:
                    msg = 'Loop time was: ' + str(time_taken) + ', sleeping for: ' + str(delay_time) + ' seconds.'
                    self.logger.log(msg)
                    sleep(delay_time)
                else:
                    self.logger.log_warn('Loop took longer than the specified period. System is not able to perform in real-time at this periodicity.')

            self.previous_state = self.current_state
            self.step = self.step + 1

if __name__ == '__main__':
    threading.Thread(target=lambda: rospy.init_node('ralt_semantic_event_logger', disable_signals=True)).start()
    pub = rospy.Publisher('ralt_semantic_event_publisher', String, queue_size=10)
    m = Main(pub)
    m.loop()