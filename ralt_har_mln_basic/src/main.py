#! /usr/bin/env python3

import os
import rospy
import rospkg
import threading
from pracmln import MLN, Database, MLNQuery
import numpy as np
from flask import Flask, request, jsonify
from flask_cors import CORS

from std_msgs import msg
from std_msgs.msg import String

from ralt_har_mln_basic.msg import simple_evidence

class Main():
    def __init__(self):
        self.sub = rospy.Subscriber('/ralt_semantic_event_logger/simple', simple_evidence, callback=self.evidence_callback)

        rospack = rospkg.RosPack()
        rel_path = rospack.get_path('ralt_har_mln_basic')
        path = rel_path + '/src/MLNs/kitchen_v1.mln'
        self.mln = MLN.load(files=path, grammar='StandardGrammar', logic='FuzzyLogic')
        self.db = Database(self.mln)

        self.events = ['Kettle', 'Tap', 'Oven', 'Fridge', 'DinnerwareCupboard', 'DrinkwareCupboard', 'FoodCupboard', 'CutleryDrawer', 'Bin']
        self.objects = ['Plate', 'Bowl', 'JuiceCarton']
        self.classes = ['PrepareColdMeal', 'PrepareHotDrink', 'PrepareHotMeal', 'PrepareColdDrink', 'WashingUp']

        self.init_objects()

    # Init. Methods

    def init_objects(self):
        for obj in self.objects:
            predicate = 'involves_object(A,' + obj + ')'
            self.db << predicate
            self.db[predicate] = 0.0

    # ROS Methods

    def loop(self):
        while(True):
            while not rospy.core.is_shutdown():
                self.decay()
                rospy.rostime.wallsleep(0.5)
            
    def evidence_callback(self, msg):
        if msg.etype == 'event':
            if msg.evidence in self.events:
                self.add(msg.evidence, msg.etype)
        elif msg.etype == 'object':
            if msg.evidence in self.objects:
                self.add(msg.evidence, msg.etype)
        else:
            pass # there is nothing we can do here to return an error really

    # API Methods
    
    def reset(self):
        del self.db # unsafe behaviour, subscriber callback may write to db between these two function calls
        self.db = Database(self.mln)

    def reason(self):
        result = MLNQuery(mln=self.mln, db=self.db, method='EnumerationAsk').run()

        queries = []
        for cla in self.classes:
            predicate = 'class(A,' + cla + ')'
            queries.append(predicate)

        probs = []
        for q in queries:
            prob = result.results[q]
            probs.append(prob)
        
        winner = np.argmax(probs)
        
        return self.classes[winner], probs[winner]

    def add(self, evidence, etype):
        resp = 'OK'

        if etype == 'event':
            if evidence in self.events:
                predicate = 'involves_event(A,' + evidence + ')'
                self.db << predicate
            else:
                resp = 'Evidence not modelled in MLN.'
        elif etype == 'object':
            if evidence in self.objects:
                predicate = 'involves_object(A,' +evidence + ')'
                self.db << predicate
            else:
                resp = 'Evidence not modelled in MLN.'
        else:
            resp = 'Invalid evidence type. Valid types are: event or object'

        return resp

    def delete(self, evidence, etype):
        resp = 'OK'

        # simple model of deletion, instantly removes, need to implement
        # (i) instant deletion for events
        # (ii) time-based deletion for objects (e.g. in 60 seconds rectractall will be called for predicate, use queue)
        if etype  == 'event':
            if evidence in self.events:
                predicate = 'involves_event(A,' + evidence + ')'
                self.db.retract(predicate)
            else:
                resp = 'Evidence not modelled in MLN.'
        elif etype == 'object':
            if evidence in self.objects:
                predicate = 'involves_object(A,' + evidence + ')'
                self.db.retract(predicate)
            else:
                resp = 'Evidence not modelled in MLN.'
        else:
            resp = 'Invalid evidence type. Valid types are: event or object' 

        return resp

    # Additional Logic

    def decay(self):
        # do something here to decay events, e.g.
        # (i) events when added are put into a dictionary with a timestamp
        # (ii) every tick, we compare the timestamp of events with current timestamp
        # (iii) if timestamp is older than x (used a tiered system), then call delete on that event
        return

if __name__ == '__main__':
    threading.Thread(target=lambda: rospy.init_node('ralt_har_mln_basic', disable_signals=True)).start()

    m = Main()

    app = Flask(__name__)
    CORS(app)

    threading.Thread(target=lambda: m.loop()).start()

    @app.route('/reset', methods = ['POST'])
    def reset_handler():

        m.reset()

        return 'OK'

    @app.route('/reason', methods = ['POST'])
    def reason_handler():

        pred, prob = m.reason()

        result = {}
        result['pred'] = pred
        result['prob'] = prob

        return jsonify(result)

    @app.route('/add', methods = ['POST'])
    def add_handler():
        data = request.get_json()

        evidence = data['evidence']
        etype = data['etype']

        resp = m.add(evidence, etype)

        return resp

    @app.route('/delete', methods = ['POST'])
    def delete_handler():
        data = request.get_json()

        evidence = data['evidence']
        etype = data['etype']

        resp = m.delete(evidence, etype)

        return resp

    app.run(host='0.0.0.0', port = 5010)