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
#
import rospkg
import os
import roslaunch
import rosgraph
import time
import uuid
import re
from multiprocessing import Process

import core.model.node as node
import core.model.param as param


class Stack(object):

    def __init__(self, composer, manifest={}, parent=None):
        self.pkg = rospkg.RosPack()
        self._manifest = manifest
        self.parent = parent
        self.composer = composer
        self._name = manifest.get('name', '')
        self._context = manifest.get('context', '')
        self._stackId = manifest.get('stackId', '')
        self._param = manifest.get('param', [])

        self.master = rosgraph.Master("master")
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.config = roslaunch.config.ROSLaunchConfig()
        self.launch_runner = roslaunch.launch.ROSLaunchRunner(
            run_id=self.uuid, config=self.config)
        self.anon = {}
        self.arg = manifest.get('arg', [])
        self.arg = self.resolveArgs(self.arg)

        ns = self.resolve_namespace()
        params = []
        for pDef in self._param:
            pns = pDef.get('namespace', '')
            pDef['namespace'] = self.resolve_namespace(pns)
            params.append(param.Param(self, pDef))
        self._param = params
        self.initialize()

    @property
    def param(self):
        return self._param

    @property
    def manifest(self):
        return self._manifest

    @property
    def node(self):
        return self._node

    @property
    def stack(self):
        return self._stack

    def resolve_namespace(self):
        return '/'

    def resolve_namespace(self, pns=""):
        ns = '/'
        if not ns.startswith('/'):
            ns = '/' + ns
        if ns.endswith('/'):
            return ns + pns + '/'
        return ns + '/' + pns + '/'

    def get_master(self):
        return self.master

    def get_launchRunner(self):
        return self.launch_runner

    def initialize(self):
        self._stack = []
        referencedStacks = self.manifest.get('stack', [])
        for stackRef in referencedStacks:
            stackDef = self.composer.stack(stackRef['thingId'])
            stack = Stack(self.composer, stackDef, self)
            self._stack.append(stack)

        self._node = []
        for nDef in self.manifest.get('node', []):
            sn = node.Node(self, nDef)
            self._node.append(sn)

    def compare(self, other):
        nodeSet = set(self.flattenNodes([]))
        otherNodeSet = set(other.flattenNodes([]))
        common = nodeSet.intersection(otherNodeSet)
        difference = nodeSet.difference(otherNodeSet)
        added = otherNodeSet.difference(nodeSet)
        return common, difference, added

    def flattenNodes(self, list):
        for n in self._node:
            list.append(n)
        for s in self._stack:
            s.flattenNodes(list)
        return list

    def merge(self, other):
        common, difference, added = self.compare(other)
        for n in common:
            n.action = 'none'
        for n in added:
            n.action = 'start'
        for n in difference:
            n.action = 'stop'
        all = common.union(added).union(difference)
        myParams = {}
        self.load_params(myParams)

        otherParams = {}
        other.load_params(otherParams)

        merged = Stack(self.composer, manifest={}, parent=None)

        merged._name = other._name
        merged._context = other._context
        merged._stackId = other._stackId

        merged._stack = []
        merged._node = all
        merged._param = []
        for pn, pv in otherParams.items():
            merged._param.append(param.Param(self, {"name": pn, "value": pv}))
        merged.arg = other.arg
        merged._manifest = merged.toManifest()
        return merged

    def toShallowManifest(self):
        manifest = {"name": self._name,
                    "context": self._context, 
                    "stackId": self._stackId, 
                    "param": [], 
                    "arg": [], 
                    "stack": [], 
                    "node": []}
        return manifest

    def toManifest(self):
        manifest = self.toShallowManifest()
        for p in self._param:
            manifest["param"].append(p.toManifest())
        for a in self.arg:
            manifest["arg"].append(self.arg[a])
        for s in self._stack:
            manifest["stack"].append(s.toShallowManifest())
        for n in self._node:
            manifest["node"].append(n.toManifest())
        return manifest

    def launch(self):
        self.launchMaster()
        self.load_params()
        for s in self._stack:
            s.launch()
        for n in self._node:
            n.launch()

    def apply(self):
        self.launchMaster()
        self.load_params()
        for s in self._stack:
            s.apply()
        for n in self._node:
            n.apply()

    def kill(self):
        for s in self._stack:
            try:
                s.kill()
                print(" %s done." % s.manifest.get('stackId', 'stack'))
            except:
                print("Stack could not be killed ")
        for n in self._node:
            try:
                n.kill()
                print(' %s done.' % n.name)
            except:
                print("Node could not be killed ")

    def load_params(self, collector=None):
        for p in self._param:
            p.load(collector)
        if not collector is None:
            for s in self._stack:
                s.load_params(collector)
            for n in self._node:
                n.load_params(collector)

    def launchMaster(self):
        if not self.master.is_online():
            print("master is not running, launching now")
            time.sleep(5)
            def callback(launcher): return self.on_launch_master(launcher)
            p = Process(target=callback, args=(self.launch_runner,))
            p.start()
            p.join()

    def on_launch_master(self, launcher):
        proc, success = launcher._launch_master()
        print("Master launched:", proc, success)

    def resolveExpression(self, v=""):
        value = str(v)
        if value is None:
            return value
        expressions = re.findall('\$\(([\s0-9a-zA-Z_-]+)\)', value)
        vals = []
        for match in expressions:
            expr, var = match.split()
            if expr == 'find':
                vals.append(self.pkg.get_path(var))
            elif expr == 'env':
                v = os.environ.get(var)
                if v is None:
                    raise Exception(value + ' does not exist', 'param')
                vals.append(v)
            elif expr == 'optenv':
                defv = var
                if defv is None:
                    defv = ''
                # return re.sub('\$\((.*)\)', os.environ.get(var,defv) , value)
                vals.append(os.environ.get(var, defv))

            elif expr == 'arg':
                a = self.arg.get(var)
                vals.append(a['value'])
            elif expr == 'anon':
                if var in self.anon.keys():
                    return self.anon[var]
                self.anon['key'] = var + uuid.uuid1().hex
                # return re.sub('\$\((.*)\)', self.anon['key'] , value)
                vals.append(self.anon['key'])
            elif expr == 'eval':
                raise Exception(value + ' NOT SUPPORTED IN MUTO', 'param')
            else:
                return value
        result = value
        for v in vals:
            result = re.sub('\$\(([\s0-9a-zA-Z_-]+)\)', v, result, count=1)
        return result

    def resolveParamExpression(self, param={}):
        result = {}
        name = ''
        value = None
        valKey = None
        for k in param.keys():
            if 'name' == k:
                name = param['name']
            else:
                value = param[k]
                valKey = k
        if not valKey is None:
            return (name, valKey, self.resolveExpression(value))
        return None

    def resolveExpressions(self, array=[]):
        result = {}
        for item in array:
            name, key, value = self.resolveParamExpression(item)
            p = {"name": name}
            p[key] = value
            result[name] = p
        return result

    def resolveArgs(self, array=[]):
        result = {}
        self.arg = {}
        for item in array:
            name, key, value = self.resolveParamExpression(item)
            p = {"name": name}
            p[key] = value
            result[name] = p
            self.arg[name] = p

        return result
