#! /usr/bin/env python

import rospy
import threading
import socket

from std_msgs.msg import String
from csv_tools import CSVTools
import time

HOST = '0.0.0.0'
PORT = 6000

if __name__ == '__main__':
    threading.Thread(target=lambda: rospy.init_node('aw_pub', disable_signals=True)).start()
    pub = rospy.Publisher('apple_watch_publisher', String, queue_size=10)

    csvt = CSVTools()
    csvt.create_event_file()

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    s.bind((HOST, PORT))
    s.listen(1)

    count = 0

    while True:
        conn, client = s.accept()
        try:
            while True:
                data = conn.recv(1024)
                if data:
                    print(data)
                    message = str(data)
                    
                    timestamp_ms = time.time()
                    timestamp_formatted = time.ctime(timestamp_ms)

                    event = [data, timestamp_ms, timestamp_formatted]
                    events = [event]
                    csvt.write_events(events)

                if not rospy.is_shutdown():
                        pub.publish(message)
                else:
                    break
        finally:
            conn.close()
