#! /usr/bin/env python
from rosbag.bag import Bag
import rospy
import threading
from time import time, strftime, sleep, ctime
import subprocess
from flask import Flask, request, jsonify
from flask_cors import CORS

from std_msgs import msg
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image, CompressedImage

from bag_merge import merge_bag
from check_create_folder_tool import FolderCheckCreate
from csv_tools import CSVTools

class Main():
    def __init__(self):
        self.id = 'main'

        self.date_time = strftime("%Y%m%d-%H%M%S")

        self.topics_robot_1 = []
        self.topics_robot_2 = []
        self.topics_other = []
        self.combined_topics = []

        self.run = False
        self.bag_name = 'No current or previous bag has been saved.'
        self.merged_bag_name = 'No bag files have been merged.'

        self.activity = 'none'

        self.participant = 999
        
        self.object_queue = []
        
        self.csv_created = False

        self.load_topics()

        self.act_pub = rospy.Publisher('/activity_label', String, queue_size=10)
        self.obj_pub = rospy.Publisher('/object_label', String, queue_size=10)

        print('Ready.')

    def load_topics(self):
        with open('/home/sandbox/config/topics_robot_1.txt') as f:
            lines = f.readlines()

            lines_clean = []
            for line in lines:
                lines_clean.append(line.rstrip('\n'))

            for line in lines_clean:
                self.topics_robot_1.append(line)
            
            print('Will check for messages on these topics for Robot 1...')
            print(self.topics_robot_1)

        with open('/home/sandbox/config/topics_robot_2.txt') as f:
            lines = f.readlines()

            lines_clean = []
            for line in lines:
                lines_clean.append(line.rstrip('\n'))

            for line in lines_clean:
                self.topics_robot_2.append(line)
            
            print('Will check for messages on these topics for Robot 2...')
            print(self.topics_robot_2)

        with open('/home/sandbox/config/topics_other.txt') as f:
            lines = f.readlines()

            lines_clean = []
            for line in lines:
                lines_clean.append(line.rstrip('\n'))

            for line in lines_clean:
                self.topics_other.append(line)
            
            print('Will also check for messages on these topics (must be on same master as Robot 1)...')
            print(self.topics_other)

        self.combined_topics = self.topics_robot_1 + self.topics_other
            
    def loop(self):
        while(True):
            if self.run:               
                date_time = strftime("%Y%m%d-%H%M%S")

                if self.activity == 'none':
                    self.bag_name = '/home/sandbox/output/' + 'P' + str(self.participant) + '/ROSbag_' + date_time + '.bag'
                else:
                    self.bag_name = '/home/sandbox/output/' + 'P' + str(self.participant) + '/ROSbag_' + date_time + '_' + self.activity + '.bag'

                while(self.run):
                    cmd = ['rosbag', 'record', '-O', self.bag_name]
                    for topic in self.combined_topics:
                        cmd.append(topic)
                    
                    self.p = subprocess.Popen(cmd)

                    while not rospy.core.is_shutdown() and self.run:
                        self.act_pub.publish(self.activity)
                        if self.object_queue:
                            object = self.object_queue.pop(0)
                            self.obj_pub.publish(object)
                        rospy.rostime.wallsleep(0.5)

                self.p.terminate()            

            sleep(1)

    def set_state(self, cmd):
        if cmd:
            self.run = True
        else:
            self.run = False
            self.csv_created = False

    def set_activity(self, activity):
        self.activity = activity
        # timestamp_s = time()
        timestamp_s = rospy.Time.now()
        timestamp_s = timestamp_s.secs
        timestamp_formatted = ctime(timestamp_s)
        labels = [self.activity, 'n/a', timestamp_s, timestamp_formatted]
        if not self.csv_created:
            self.csv_tools = CSVTools()
            self.csv_tools.create_labels_file(self.activity, self.participant)
            self.csv_created = True
        self.csv_tools.write_labels([labels])
            
    def set_participant(self, participant):
        self.participant = participant
        fcc = FolderCheckCreate()
        fcc.run(self.participant)
        
    def register_object(self, object):
        self.object_queue.append(object)
        # timestamp_s = time()
        timestamp_s = rospy.Time.now()
        timestamp_s = timestamp_s.secs
        timestamp_formatted = ctime(timestamp_s)
        labels = [self.activity, self.object_queue[-1], timestamp_s, timestamp_formatted]
        if not self.csv_created:
            self.csv_tools = CSVTools()
            self.csv_tools.create_labels_file(self.activity, self.participant)
            self.csv_created = True
        self.csv_tools.write_labels([labels])

    def set_merged_bag_name(self, name):
        self.merged_bag_name = name

if __name__ == '__main__':
    threading.Thread(target=lambda: rospy.init_node('ralt_signalman', disable_signals=True)).start()

    m = Main()

    app = Flask(__name__)
    CORS(app)

    threading.Thread(target=lambda: m.loop()).start()
    
    @app.route('/control', methods = ['POST'])
    def control_handler():
        data = request.get_json()
        
        command = data['command']
        activity = data['activity']

        print(command, activity)

        resp = "OK"

        m.set_activity(activity)

        if command == "True":
            m.set_state(True)
        elif command == "False":
            m.set_state(False)
        else:
            resp = "Invalid state. Send command to either 'True' or 'False'."

        return resp

    @app.route('/participant', methods = ['POST'])
    def participant_handler():
        data = request.get_json()

        participant = data['participant']

        participant = int(participant)

        m.set_participant(participant)

        resp = "OK"

        return resp

    @app.route('/status', methods = ['GET'])
    def status_handler():
        status = {}

        status["topics_robot_1"] = m.topics_robot_1
        status["topics_robot_2"] = m.topics_robot_2
        status["topics_other"] = m.topics_other
        status["running"] = m.run
        status["bagfile"] = m.bag_name
        status["merged_bagfile"] = m.merged_bag_name

        return jsonify(status)

    @app.route('/update', methods = ['POST'])
    def update_handler():
        data = request.get_json()

        activity = data['activity']

        resp = "OK"

        m.set_activity(activity)

        return resp
    
    @app.route('/object', methods = ['POST'])
    def object_handler():
        data = request.get_json()

        object = data['object']

        resp = "OK"

        m.register_object(object)

        return resp

    @app.route('/merge', methods = ['POST'])
    def merge_handler():
        data = request.get_data()
        
        resp = "OK"

        main_bag = m.bag_name
        
        secondary_bag = data

        print(main_bag, secondary_bag)

        out_bag = main_bag.strip('.bag')
        out_bag = out_bag + '_merged.bag'
        
        m.set_merged_bag_name(out_bag)

        merge_bag(main_bag, secondary_bag, outfile=out_bag) 

        return jsonify(resp)

    app.run(host='0.0.0.0', port = 5002)
