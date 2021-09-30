
from time import strftime
import csv

class CSVTools():
    def __init__(self):
        self.id = 'csv_tools'

    def create_event_file(self):
        date_time = strftime("%Y%m%d-%H%M%S")
        self.csv_filename = '/home/sandbox/shared/output/apple_watch_' + date_time + '.csv'

        msg = 'The events CSV file for this session is: ' + self.csv_filename
        print(msg)

        with open(self.csv_filename, 'w') as fd:
            writer = csv.writer(fd)
            writer.writerow(['data', 'timestamp (ms)', 'timestamp (formatted)'])

    def write_events(self, events):
        with open(self.csv_filename, 'a') as fd:
            writer = csv.writer(fd)
            writer.writerows(events)
