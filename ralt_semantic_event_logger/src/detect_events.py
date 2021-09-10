import time
from datetime import datetime
import copy

from log import Log

class DetectEvents():
    def __init__(self, sensors, sensor_labels):
        self.id = 'detect_events'

        self.logger = Log(self.id)

        self.sensors = sensors
        self.sensor_labels = sensor_labels

        self.logger.log_great('Ready.')

    def init_semantic_state(self, current):
        for key, value in current.items():
            current_state = current[key]['state']
            previous_state = current_state
            
            semantic_state = ''
            inverted = self.sensors[key]['inverted']

            if self.sensors[key]['reports'] == 'binary':
                semantic_state = self.handle_binary_change(current_state, inverted)
            elif self.sensors[key]['reports'] == 'numerical':
                semantic_state = self.handle_numerical_change(current_state, previous_state, inverted)
            elif self.sensors[key]['reports'] == 'numerical_threshold':
                semantic_state = self.handle_numerical_threshold_change(current_state, inverted, self.sensors[key]['threshold'])
            elif self.sensors[key]['reports'] == 'energy':
                semantic_state = self.handle_energy_change(current_state, previous_state, self.sensors[key]['threshold'])
            else:
                msg = 'Invalid sensor type. No method to handle: ' + key
                self.logger.log_warn(msg)
                semantic_state = 'invalid sensor type'
            
            self.sensors[key]['semantic_state'] = semantic_state
            self.sensors[key]['raw_state'] = current_state

    def step(self, current, previous, step_count):
        self.timestamp_ms = time.time()
        self.timestamp_formatted = time.ctime(self.timestamp_ms)
        self.step_count = step_count

        events = []
        
        filtered_raw = self.check_for_changes_raw(current, previous)

        if not bool(filtered_raw):
            return events

        sensors_ds_for_comparison = copy.deepcopy(self.sensors)

        for key, value in filtered_raw.items():
            current_state = current[key]['state']
            previous_state = previous[key]['state']
            
            semantic_state = ''
            inverted = self.sensors[key]['inverted']

            if self.sensors[key]['reports'] == 'binary':
                semantic_state = self.handle_binary_change(current_state, inverted)
            elif self.sensors[key]['reports'] == 'numerical':
                semantic_state = self.handle_numerical_change(current_state, previous_state, inverted)
            elif self.sensors[key]['reports'] == 'numerical_threshold':
                semantic_state = self.handle_numerical_threshold_change(current_state, inverted, self.sensors[key]['threshold'])
            elif self.sensors[key]['reports'] == 'energy':
                semantic_state = self.handle_energy_change(current_state, previous_state, self.sensors[key]['threshold'])
            else:
                msg = 'Invalid sensor type. No method to handle: ' + key
                self.logger.log_warn(msg)
                semantic_state = 'invalid sensor type'
            
            self.sensors[key]['semantic_state'] = semantic_state
            self.sensors[key]['raw_state'] = current_state

        filtered_semantic = self.check_for_changes_semantic(sensors_ds_for_comparison)

        if not bool(filtered_semantic):
            return events

        events = self.generate_events(filtered_semantic)
        print(events)

        return events

    def check_for_changes_raw(self, current, previous):
        filtered = {}
        for key, value in current.items():
            if value['state'] != 'UNDEF' and previous[key]['state'] != 'UNDEF':
                if value['state'] != previous[key]['state']: # this should be != in deployment!
                    filtered[key] = value
            else:
                print('[WARN] Sensor', key, 'has entered an undefined state.')

        return filtered

    def check_for_changes_semantic(self, compare):
        filtered = {}
        for key, value in self.sensors.items():
            if value['semantic_state'] != compare[key]['semantic_state']:
                filtered[key] = value

        return filtered

    def handle_binary_change(self, current, inverted):
        semantic_state = ''

        if not inverted:
            if current == 'ON':
                semantic_state = 'true'
            elif current == 'OFF':
                semantic_state = 'false'
            else:
                semantic_state = 'undefined'
        else:
            if current == 'ON':
                semantic_state = 'false'
            elif current == 'OFF':
                semantic_state = 'true'
            else:
                semantic_state = 'undefined'
        
        return semantic_state

    def handle_numerical_change(self, current, previous, inverted):
        semantic_state = ''
        current = int(current)
        previous = int(previous)

        if not inverted:
            if current > previous:
                semantic_state = 'down'
            else:
                semantic_state = 'up'
        else:
            if current > previous:
                semantic_state = 'up'
            else:
                semantic_state = 'down'

        return semantic_state

    def handle_numerical_threshold_change(self, current, inverted, threshold):
        semantic_state = ''
        current = int(current)

        if not inverted:
            if current > threshold:
                semantic_state = 'true'
            else:
                semantic_state = 'false'
        else:
            if current > threshold:
                semantic_state = 'false'
            else:
                semantic_state = 'true'

        return semantic_state

    def handle_energy_change(self, current, previous, threshold):
        semantic_state = ''

        current = current.strip('W')
        previous = previous.strip('W')

        current = int(float(current))
        previous = int(float(previous))
        
        if current > threshold:
            semantic_state = 'true'
        else:
            semantic_state = 'false'

        return semantic_state

    def generate_events(self, filtered):
        events = []

        for key, value in filtered.items():
            event_type = self.sensors[key]['event']
            origin = key
            raw = self.sensors[key]['raw_state']
            timestamp_ms = self.timestamp_ms
            timestamp_formatted = self.timestamp_formatted
            step_count = self.step_count

            if self.sensors[key]['logic']['predicate'] == 'event':
                predicate = self.sensors[key]['event']
                x = self.sensors[key]['logic']['x']
                y = self.sensors[key]['logic']['y']
                predicate_xy = predicate + '(e)'
            else:
                if self.sensors[key]['alt_predicate_when_true'] == 'true':
                    if raw == 'true':
                        predicate = self.sensors[key]['logic']['alt_predicate']
                    else:
                        predicate = self.sensors[key]['logic']['predicate']
                else:
                    predicate = self.sensors[key]['logic']['predicate']

                if self.sensors[key]['logic']['y'] == 'state':
                    x = self.sensors[key]['logic']['x']
                    y = self.sensors[key]['semantic_state']
                    predicate_xy = predicate + '(' + x + ',' + y + ')'
                else:
                    x = self.sensors[key]['logic']['x']
                    y = self.sensors[key]['logic']['y']
                    predicate_xy = predicate + '(' + x + ',' + y + ')'

            event = [predicate, x, y, predicate_xy, event_type, origin, raw, timestamp_ms, timestamp_formatted, step_count]

            if self.sensors[key]['ignore_false'] != 'true':
                events.append(event)
            elif self.sensors[key]['ignore_false'] == 'true' and key == 'true':
                events.append(event)

        return events
