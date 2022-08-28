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
import core.model.stack as stack

UNKNOWN = 'unknown'
ACTIVE = 'active'
KILLED = 'killed'


class EdgeDevice(object):

    def __init__(self, twin, params={}):
        self._params = params
        self.twin = twin
        self._definition = None
        self._state = None
        self._currentStack = None

    def connect(self):
        try:
            self.twin.register()
        except:
            print("An exception occurred in device registration")

    def bootstrap(self):
        try:
            currentDefinition = self.twin.get_current_stack()
            if currentDefinition is None:
                currentDefinition = {}
            currentDefinition = currentDefinition.get('current', None)
            if not currentDefinition is None:
                self._definition = self.stack(
                    currentDefinition.get('stackId', {}))
                self._state = currentDefinition.get('state', UNKNOWN)
            if not self._definition is None:
                self._currentStack = stack.Stack(
                    self, self._definition, parent=None)
            # if ACTIVE == self._state:
            #    self.activate()
        except:
            print("An exception occurred in device bootstrapping")

    def activate(self, current=None):
        if not current is None:
            self._currentStack = stack.Stack(self, current, None)
        self._currentStack.launch()
        self.twin.publishStack(self._currentStack, state=ACTIVE)
        self._state = ACTIVE

    def apply(self, current=None):
        if not current is None:
            self._currentStack = stack.Stack(self, current, None)
        self._currentStack.apply()
        self.twin.publishStack(self._currentStack, state=ACTIVE)
        self._state = ACTIVE

    def kill(self, payload=None):
        if self._currentStack:
            self._currentStack.kill()
        if not payload is None:
            self._currentStack = stack.Stack(self, payload, None)
            self._currentStack.kill()
        self.twin.publishStack(self._currentStack, state=KILLED)
        self._state = KILLED

    def stack(self, stackId):
        return self.twin.stack(stackId)

    @property
    def params(self):
        return self._params

    @params.setter
    def params(self, n):
        self._params = n

    @property
    def topic(self):
        return self.twin.topic

    @topic.setter
    def topic(self, n):
        self.twin.topic = n

    @property
    def definition(self):
        return self._definition

    @definition.setter
    def definition(self, n):
        self._definition = n

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, n):
        self._state = n

    @property
    def currentStack(self):
        return self._currentStack

    @currentStack.setter
    def currentStack(self, n):
        self._currentStack = n
