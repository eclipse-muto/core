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
import rosparam
import subprocess
from roslib.names import ns_join


class Param(object):

    def __init__(self, stack, manifest={}):
        self.stack = stack
        self._manifest = manifest
        self._param = manifest.get('param', [])  # recursive

        self._name = manifest.get('name', '')
        self._value = manifest.get('value', '')
        self._sep = manifest.get('sep', '')
        self._from_file = manifest.get('from', '')
        self._namespace = manifest.get('namespace', '/')
        self._command = manifest.get('command', '')

        if self._from_file:
            self.from_file = stack.resolveExpression(self.from_file)
        elif self._command:
            self.value = stack.resolveExpression(self._command)
        elif 'value' in manifest.keys():
            self.value = stack.resolveExpression(self.value)

    def toManifest(self):
        manifest = {"name": self._name, "value": self._value,
                    "sep": self._sep, "from": self._from_file,
                    "namespace": self._namespace,
                    "command": self._command, "param": []}
        for p in self._param:
            manifest["param"].append(p.toManifest())
        return manifest

    def __eq__(self, other):
        """Overrides the default implementation"""
        if isinstance(other, Param):
            if self.name != other.name:
                return False
            if self.value != other.value:
                return False
            if self.from_file != other.from_file:
                return False
            if self.namespace != other.namespace:
                return False
            if self.command != other.command:  # no getter setter
                return False
            return True
        return False

    def __ne__(self, other):
        """Overrides the default implementation (unnecessary in Python 3)"""
        return not self.__eq__(other)

    def load(self, collector=None):
        if self.from_file:
            param_dict = rosparam.load_file(self.from_file, self.namespace)
            for v, k in param_dict:
                if not collector is None:
                    for pname, pvalue in v.items():
                        collector[ns_join(k, pname)] = pvalue
                else:
                    rosparam.upload_params(k, v)
        elif self._command:
            v = subprocess.check_output(self.value.split())
            if not collector is None:
                collector[ns_join(self.namespace , self.name)] = v.decode("utf-8")
            else:
                rosparam.set_param(self.namespace + self.name,v.decode("utf-8"))
        else:
            if not collector is None:
               collector[ns_join(self.namespace , self.name)] = self.value
            else:
                rosparam.set_param(self.namespace + self.name, self.value)

    @property
    def manifest(self):
        return self._manifest

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, n):
        self._name = n

    @property
    def namespace(self):
        return self._namespace

    @namespace.setter
    def namespace(self, n):
        self._namespace = n

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, n):
        self._value = n

    @property
    def sep(self):
        return self._sep

    @sep.setter
    def sep(self, n):
        self._sep = n

    @property
    def from_file(self):
        return self._from_file

    @from_file.setter
    def from_file(self, n):
        self._from_file = n

    @property
    def command(self):
        return self._command

    @command.setter
    def command(self, n):
        self._command = n
