from time import strftime
import csv

from log import Log

class CSVTools():
    def __init__(self):
        self.id = 'csv_tools'

        self.logger = Log(self.id)

        self.logger.log_great('Ready.')

    def create_event_file(self, activity):
        date_time = strftime("%Y%m%d-%H%M%S")

        if activity == 'none':
            self.csv_filename = '/home/sandbox/shared/output/events_' + date_time + '.csv'
        else:
            self.csv_filename = '/home/sandbox/shared/output/events_' + date_time + '_' + activity + '.csv'

        msg = 'The events CSV file for this session is: ' + self.csv_filename
        self.logger.log(msg)

        with open(self.csv_filename, 'w') as fd:
            writer = csv.writer(fd)
            writer.writerow(['predicate', 'x', 'y', 'predicate(x,y)', 'event', 'origin', 'raw', 'timestamp (ms)', 'timestamp (formatted)', 'index'])

    def write_events(self, events):
        with open(self.csv_filename, 'a') as fd:
            writer = csv.writer(fd)
            writer.writerows(events)