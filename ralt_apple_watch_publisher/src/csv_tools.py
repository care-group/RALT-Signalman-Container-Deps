
from time import strftime
import csv

class CSVTools():
    def __init__(self):
        self.id = 'csv_tools'

    def create_event_file(self, activity, participant):
        date_time = strftime("%Y%m%d-%H%M%S")

        if activity == 'none':
            self.csv_filename = '/home/sandbox/output/' + 'P' + str(participant) + '/apple_watch_' + date_time + '.csv'
        else:
            self.csv_filename = '/home/sandbox/output/' + 'P' + str(participant) + '/apple_watch_' + date_time + '_' + activity + '.csv'

        msg = 'The events CSV file for this session is: ' + self.csv_filename
        print(msg)

        with open(self.csv_filename, 'w') as fd:
            writer = csv.writer(fd)
            writer.writerow(['data', 'timestamp (s)', 'timestamp (formatted)'])

    def write_events(self, events):
        with open(self.csv_filename, 'a') as fd:
            writer = csv.writer(fd)
            writer.writerows(events)
