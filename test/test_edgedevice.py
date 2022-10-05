#! /usr/bin/env python
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

import unittest

import rospy

import core.model.node as node
import core.model.param as param
import core.model.stack as stack
import core.ditto.twin as twin
import core.model.edge_device as edge_device

from unittest.mock import patch
from sample_data import TestSample

PKG = 'muto_core'
NODE_NAME = 'muto_core'


class TestEdgeDeviceModel(unittest.TestCase):

    @patch('core.ditto.twin.Twin')
    def test_device_register(self, mock_twin):
        samples = TestSample()

        mock_twin.get_current_stack.return_value = { "current": { "stackId": "mock_stack_1", "state": "active"}}
        mock_twin.stack.return_value = samples.exampleStackA()
        device = edge_device.EdgeDevice(params={}, twin=mock_twin)
        device.connect()
        mock_twin.register.assert_called_with()
        self.assertIsNotNone(device)   
        device.bootstrap()
        mock_twin.get_current_stack.assert_called_with()
        mock_twin.stack.assert_called_with("mock_stack_1")

        device.activate(current=None)

        mock_twin.publishStack.assert_called_with(device.currentStack, state="active")
        device.kill(payload=None)





if __name__ == '__main__':
    import rostest

    rostest.rosrun(
        PKG,
        'Test Edge Device',
        TestEdgeDeviceModel
    )


