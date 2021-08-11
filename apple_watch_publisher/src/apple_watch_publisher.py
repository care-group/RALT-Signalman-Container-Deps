#! /usr/bin/env python

import rospy
import json
import pymongo
import threading
import socket

from std_msgs.msg import String

HOST = '0.0.0.0'
PORT = 5000

mongodb_client = pymongo.MongoClient("mongodb://localhost:27017/")
target_db = mongodb_client["watch_data"]
target_col = target_db["stream"]

if __name__ == '__main__':
    threading.Thread(target=lambda: rospy.init_node('aw_pub', disable_signals=True)).start()
    pub = rospy.Publisher('apple_watch_publisher', String, queue_size=10)

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

                    print(count)
                    count = count + 1

                    # x = target_col.insert_one(json_data)

                    if not rospy.is_shutdown():
                        pub.publish(message)
                else:
                    break
        finally:
            conn.close()
