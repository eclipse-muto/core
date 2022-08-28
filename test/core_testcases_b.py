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
#! /usr/bin/env python

import unittest

import rospy

import core.model.node as node
import core.model.param as param
import core.model.stack as stack
from unittest.mock import patch
from sample_data import TestSample

PKG = 'muto_core'
NODE_NAME = 'muto_core'


class TestNodeModel(unittest.TestCase):

    @patch('core.model.edge_device.EdgeDevice')
    def test_node_basic(self, mock_device):
        samples = TestSample()
        simpleNode = samples.sampleNode(mock_device)

        # do some things to my_var which might change its value...
        self.assertEquals("Particle_filter", simpleNode.name)
        simpleNode.name = "Another Name"
        self.assertEquals("Another Name", simpleNode.name)
        
    @patch('core.model.edge_device.EdgeDevice')
    def test_node_with_arg_param(self, mock_device):
        samples = TestSample()
        simpleNode = samples.sampleNodeWithArgParam(mock_device)

        # do some things to my_var which might change its value...
        self.assertEquals("Particle_filter", simpleNode.name)
        simpleNode.name = "Another Name"
        self.assertEquals("Another Name", simpleNode.name)

        for p in simpleNode.param:
            print(p)
            if p.name == 'test_param':
                self.assertAlmostEquals('test_param', p.name)
                self.assertAlmostEquals('racecar-v2', p.value)
        


if __name__ == '__main__':
    import rostest

    rostest.rosrun(
        PKG,
        'Test Stack Nodes',
        TestNodeModel
    )


