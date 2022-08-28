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

class Arg(object):

    def __init__(self, dict={}):
        self._name = dict.get('name', '')
        self._value = dict.get('value', '')
        self._default = dict.get('default', '')
        self._description = dict.get('description', '')

    def toManifest(self):
        manifest = {"name": self._name, "value": self._value,
                    "default": self._default, "description": self._description}
        return manifest

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, n):
        self._name = n

    @property
    def value(self):
        if not self._value is None:
            return self._value
        return self._default

    @value.setter
    def value(self, n):
        self._value = n

    @property
    def default(self):
        return self._default

    @default.setter
    def default(self, n):
        self._default = n

    @property
    def description(self):
        return self._description

    @description.setter
    def description(self, n):
        self._description = n
