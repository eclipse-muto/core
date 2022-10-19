#
#  Copyright (c) 2022 Composiv.ai, Eteration A.S. and others
#
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# and Eclipse Distribution License v1.0 which accompany this distribution.
#
# The Eclipse Public License is available at
#    http://www.eclipse.org/legal/epl-v10.html
# and the Eclipse Distribution License is available at
#   http://www.eclipse.org/org/documents/edl-v10.php.
#
# Contributors:
#    Composiv.ai, Eteration A.S. - initial API and implementation
#

#!/usr/bin/env python2
from argparse import Namespace
import requests
import rospy


import time
import json
import uuid


class Twin(object):
    """
    The class that handles Muto Digital Twin
    """

    def __init__(self, node, config, publisher=None):

        self.twin_url = config['twin_url']
        self.publisher = publisher
        self.namespace = config.get(
            "namespace",  config.get("thing", {}).get("namespace"))
        self.type = config['type']
        uniqueName = config.get(
            "uniqueName",  config['type']+"."+str(uuid.uuid4()))

        if rospy.has_param(node+'/muto/thing/anonymous'):
            anonymous = rospy.get_param(node+"/muto/thing/anonymous")
            if anonymous is False:
                uniqueName = rospy.get_param(node+"/muto/thing/name")

        self._uniqueName = uniqueName
        self._topic = self.namespace+":" + uniqueName
        self.thingId = self.namespace + ":" + uniqueName
        self.last_publish = {}

    def getContext(self):
        context = {}
        context['namespace'] = self.namespace
        context['topic'] = self._topic
        context['twin_url'] = self.twin_url
        context['type'] = self.type
        context['uniqueName'] = self._uniqueName
        context['thingId'] = self.thingId
        return context

    @property
    def topic(self):
        return self._topic

    @topic.setter
    def topic(self, n):
        self._topic = n

    @property
    def uniqueName(self):
        return self._uniqueName

    @uniqueName.setter
    def uniqueName(self, n):
        self._uniqueName = n

    def register(self):
        # USE MERGE JSON SEMANTICS application/merge-patch+json.
        headers = {'Content-type': 'application/merge-patch+json'}
        r = requests.patch(self.twin_url + "/api/2/things/" + self.thingId,
                           headers=headers, json=self.get_register_data())
        if r.status_code == 404:
            # First time use PUT not PATCH
            r = requests.put(self.twin_url + "/api/2/things/" +
                             self.thingId, json=self.get_register_data())

        print("Status Code: %d, Response: %s" % (r.status_code, r.text))

    def get_current_stack(self):
        return self.stack(self.thingId)

    def stack(self, thingId):
        try:
            r = requests.get(self.twin_url + "/api/2/things/" +
                             thingId + '/features/stack')
            print("Status Code: %d, Response: %s" % (r.status_code, r.text))
            if r.status_code >= 300:
                return {}
            payload = json.loads(r.text)
            return payload.get('properties', {})
        except Exception as ex:
            print('Could not get stack from twins repo {}'.format(ex))
        return None

    def vehicle(self, vehicle):
        try:
            r = requests.get(self.twin_url + "/api/2/things/" + vehicle)
            print("Status Code: %d, Response: %s" % (r.status_code, r.text))
            if r.status_code >= 300:
                return None
            payload = json.loads(r.text)
            return payload
        except Exception as ex:
            print('Could not get stack from twins repo {}'.format(ex))
        return None

    def publishTelemetry(self, path, value):
        self.publishFeature(path, value, rate=5000)

    def publishStack(self, stack, state='unknown'):
        if not stack:
            return
        deftn = stack.manifest
        stackId = deftn.get('stackId', None)
        if not stackId is None:
            self.publishFeature(
                "/features/stack/properties/current", {"stackId": stackId, "state": state})
            return
        stacks = deftn.get('stack', [])
        for s in stacks:
            id = s.get('thingId', '')
            if id:
                self.publishFeature(
                    "/features/stack/properties/current", {"stackId": id, "state": state})
                
    def setCurrentStack(self, stack, state='unknown'):
        if not stack:
            return
        deftn = stack.manifest
        stackId = deftn.get('stackId', None)
        headers = {'Content-type': 'application/json'}

        if not stackId is None:
            r = requests.put(self.twin_url + "/api/2/things/{}/features/stack/properties/current".format(self.thingId),
                    headers=headers, json={"stackId": stackId, "state": state})
            print("Status Code: %d, Response: %s" % (r.status_code, r.text))  
            return 
  
        stacks = deftn.get('stack', [])
        for s in stacks:
            id = s.get('thingId', '')
            if id:
                r = requests.post(self.twin_url + "/api/2/things/{}/features/stack/properties/current".format(self.thingId),
                        headers=headers, json={"stackId": id, "state": state})
                print("Status Code: %d, Response: %s" % (r.status_code, r.text))  

    def get_telemetry(self):
            headers = {'Content-type': 'application/json'}
            r = requests.get(self.twin_url + "/api/2/things/{}/features/telemetry/properties".format(self.thingId),
                    headers=headers)
            print("Status Code: %d, Response: %s" % (r.status_code, r.text))
            payload = json.loads(r.text)
            return payload

    def save_telemetry(self, telemetry):
        if not telemetry:
            return
        deftn = telemetry.manifest
        headers = {'Content-type': 'application/json'}
        all = self.get_telemetry()
        mylist = []
        alldefs = all.get("definition")
        if not alldefs is None:
            def other(s1):
                if s1.get("topic") != deftn.get("topic"): 
                    return True 
                return False
            mylist = list(filter(other, alldefs))
        mylist.append(deftn)
        r = requests.put(self.twin_url + "/api/2/things/{}/features/telemetry/properties/definition".format(self.thingId),
                    headers=headers, json=mylist)
        print("Status Code: %d, Response: %s" % (r.status_code, r.text))  
        return 

    def delete_telemetry(self, telemetry):
        if not telemetry:
            return
        deftn = telemetry.manifest
        headers = {'Content-type': 'application/json'}
        all = self.get_telemetry()
        mylist = []
        alldefs = all.get("definition")
        if not alldefs is None:
            def other(s1):
                if s1.get("topic") != deftn.get("topic"): 
                    return True 
                return False
            mylist = list(filter(other, alldefs))
        r = requests.put(self.twin_url + "/api/2/things/{}/features/telemetry/properties/definition".format(self.thingId),
                    headers=headers, json=mylist)
        print("Status Code: %d, Response: %s" % (r.status_code, r.text))  
        return 

                
    def publishFeature(self, path, value, rate=-10000):
        if not self.publisher:
            return
        if not self.check_rate(path, rate):
            return
        mqttmsg = json.dumps(
            {
                "topic":  self.namespace+"/" + self.uniqueName + "/things/twin/commands/modify",
                "path": path,
                "value": value
            })

        self.publisher.publish(mqttmsg)

    def check_rate(self, path, rate):
        last = self.last_publish.get(path, 0)
        if not last:
            last = round(time.time() * 1000)
            self.last_publish[path] = last

        current = round(time.time() * 1000)
        if current - last < rate:
            return False
        self.last_publish[path] = current
        return True

    def get_register_data(self):
        return {
            "definition": "ai.composiv.sandbox.f1tenth.simulator:TestCar:1.0.0",
            "attributes": {
                "manufacturer": "Eteration",
                "serial": self.uniqueName,
                "type": self.type
            },
            "features": {
                "context": {
                    "properties": {
                        "name": "office-area-2"
                    }
                },
                "rosModel": {
                    "properties": {
                    }
                },
                "stack": {
                    "properties": {
                    }
                },
                "sensors": {
                    "properties": {
                        "camera": {},
                        "imu": {},
                        "controller": {}
                    }
                },
                "telemetry": {
                    "properties": {
                        "velocity": 0,
                        "steering_angle": 0,
                    }
                }
            }
        }
