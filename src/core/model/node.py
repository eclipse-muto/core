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
import roslaunch
from multiprocessing import Process
import core.model.param as param
import core.introspector.introspector as introspector


NOACTION =  'none'
STARTACTION = 'start'
STOPACTION = 'stop'

class Node(object):

    def __init__(self, stack, manifest={}):
        self.stack = stack
        self._manifest = manifest
        self._env = manifest.get('env', [])
        self._param = manifest.get('param', [])
        self._remap = manifest.get('remap', [])
        self._pkg = manifest.get('pkg', '')
        self._exec = manifest.get('exec', '')
        self._name = manifest.get('name', '')
        self._ros_args = manifest.get('ros_args', '')
        self._args = manifest.get('args', '')
        self._namespace = manifest.get('namespace', '/')
        self._launch_prefix = manifest.get('launch-prefix', None)
        self._output = manifest.get('output', 'log')
        self._iff = manifest.get('if', '')
        self._unless = manifest.get('unless', '')
        self._action = manifest.get('action', None)


        #  Defaults (ROS1)
        self._machine_name = None
        self._respawn = False
        self._respawn_delay = 0.0
        self._remap_args = None
        self._env_args = None
        self._cwd = 'node'
        self._required = False
        self._filename = self.name

        self._args = stack.resolveExpression(self._args)
        self._ros_args = stack.resolveExpression(self._ros_args)
        self._env = stack.resolveExpression(self._env)
        
        # 
        ns =  self.resolve_namespace()
        params = []
        for pDef in self._param:
            pDef['namespace'] = ns
            params.append(param.Param(stack, pDef))
        self._param = params

        self._remap_args = []
        for rm in self._remap:
            fr = stack.resolveExpression(rm['from'])
            to = stack.resolveExpression(rm['to'])
            self._remap_args.append((fr,to))

    def toManifest(self):
        manifest = {
            "env": [],
            "param": [],
            "remap": [],
            "pkg": self._pkg, 
            "exec": self._exec,
            "name": self._name, 
            "ros_args": self._ros_args,
            "args": self._args,
            "namespace": self._namespace,
            "launch-prefix": self._launch_prefix, 
            "output": self._output, 
            "if": self._iff, 
            "unless": self._unless,
            "action": self._action
            }
        for p in self._param:
            manifest["param"].append(p.toManifest())
        for r in self._remap:
                   self._remap_args = []
        for rm in self._remap_args:
            manifest["remap"].append({"from": rm[0], "to": rm[1] })
        return manifest

    def __eq__(self, other):
        """Overrides the default implementation"""
        if isinstance(other, Node):
            if self.pkg != other.pkg:
                return False 
            if self.name != other.name:
                return False 
            if self.namespace != other.namespace:
                return False 
            if self.exectbl != other.exectbl:
                return False 
            if self.args != other.args:
                return False 
            return True
        return False

    def __ne__(self, other):
        """Overrides the default implementation (unnecessary in Python 3)"""
        return not self.__eq__(other)

    def __hash__(self):
        h = 7
        if self.pkg is not None:
            h = 31 * h + hash(self.pkg)
        if self.name is not None:
            h = 31 * h + hash(self.name)
        if self.namespace is not None:
            h = 31 * h + hash(self.namespace)
        if self.exectbl is not None:
            h = 31 * h + hash(self.exectbl)
        if self.args is not None:
            h = 31 * h + hash(self.args)
        return h

    def apply(self):
        if self.action == NOACTION or self.action == STARTACTION :
            self.launch()
        if self.action == STOPACTION:
            self.kill()
        
    def launch(self):
        intrspc = introspector.Introspector()
        n = intrspc.get_node(self.resolve_namespace())
        if not n is None and n.get('connection',{}).get('pid',''):
            print("Node %s is already active"%self.resolve_namespace())
            return
        
        self.load_params()
        node = roslaunch.core.Node(
            package=self.pkg, 
            node_type=self.exectbl,
            name=self.name,
            namespace=self.namespace,
            machine_name=self._machine_name, 
            args=self.args,
            respawn=self._respawn, 
            respawn_delay=self._respawn_delay,
            remap_args=self._remap_args, 
            env_args=self._env_args, 
            output=self.output, 
            cwd=self._cwd,
            launch_prefix=self.launch_prefix,
            required=self._required, 
            filename=self._filename)

        def callback(launch_runner, node): return self.on_launch(launch_runner, node)
        node_launcher = Process(target=callback, args=(self.stack.get_launchRunner(), node))
        node_launcher.start()
        node_launcher.join()

    def kill(self):
        intrspc = introspector.Introspector()
        intrspc.kill(self.resolve_namespace())

    def on_launch(self, launch_runner, node):
        proc, success = launch_runner.launch_node(node)
        print("Node launched.", proc, success)


    def resolve_namespace(self):
        ns = self.namespace
        if not ns.startswith('/'):
            ns = '/'+ ns
        if ns.endswith('/'):
            return ns+ self.name +'/'
        return ns + '/' + self.name +'/'

    def load_params(self, collector=None):
        for p in self.param:
            p.load(collector)


    @property
    def env(self):
        return self._env

    @env.setter
    def env(self, n):
        self._env = n

    @property
    def param(self):
        return self._param

    @param.setter
    def param(self, n):
        self._param = n

    @property
    def remap(self):
        return self._remap

    @remap.setter
    def remap(self, n):
        self._remap = n

    @property
    def pkg(self):
        return self._pkg

    @pkg.setter
    def pkg(self, n):
        self._pkg = n

    @property
    def exectbl(self):
        return self._exec

    @exectbl.setter
    def exect(self, n):
        self._exec = n

    @property
    def namespace(self):
        return self._namespace

    @namespace.setter
    def namespace(self, n):
        self._namespace = n

    @property
    def ros_args(self):
        return self._ros_args

    @ros_args.setter
    def ros_args(self, n):
        self._ros_args = n

    @property
    def args(self):
        return self._args

    @args.setter
    def args(self, n):
        self._args = n

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, n):
        self._name = n

    @property
    def launch_prefix(self):
        return self._launch_prefix

    @launch_prefix.setter
    def launch_prefix(self, n):
        self._launch_prefix = n

    @property
    def output(self):
        return self._output

    @output.setter
    def output(self, n):
        self._output = n

    @property
    def iff(self):
        return self._iff

    @iff.setter
    def iff(self, n):
        self._iff = n

    @property
    def unless(self):
        return self._unless

    @unless.setter
    def unless(self, n):
        self._unless = n

    @property
    def action(self):
        return self._action
    @action.setter
    def action(self, n):
        self._action = n

    @property
    def manifest(self):
        return self._manifest
