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
#!/usr/bin/env python

import unittest

from re import S
from unittest.mock import patch
from sample_data import TestSample

import rospy

import core.model.param as param
import core.ditto.twin as twin
import core.model.edge_device as edge_device
import core.model.stack as stack
import core.model.node as node
import core.model.arg as arg

PKG = 'muto_core'
NODE_NAME = 'muto_core'


_name = 'argname'
_value = 'argvalue'
_default = 'ArgDefaultValue'
_description = 'ArgDescription'

_spec = {'name': _name,  'value': _value,
         'default': _default, 'description': _description}


class CaseA(unittest.TestCase):

    def setUp(self):
        rospy.init_node(NODE_NAME)
        self.arg = arg.Arg(_spec)

    def test_arg(self):

        self.assertIsNotNone(self.arg)
        self.assertEqual(
            self.arg.name, _spec['name'], 'arg name was {}'.format(_spec['name']))
        self.assertEqual(
            self.arg.default, _spec['default'], 'arg default was {}'.format(_spec['default']))
        self.assertEqual(
            self.arg.value, _spec['value'], 'arg value was {}'.format(_spec['value']))
        self.assertEqual(
            self.arg.description, _spec['description'], 'arg name was {}'.format(_spec['description']))
        self.arg.value = 'bar2'
        self.assertEqual(
            self.arg.value, 'bar2', 'arg value was {}'.format('bar2'))
        self.arg.description = 'test2'
        self.assertEqual(
            self.arg.description, 'test2', 'arg description was {}'.format('test2'))



    @patch('core.model.edge_device.EdgeDevice')
    def test_stack_compare(self, mock_edgedevice):

        samples = TestSample()

        withvesc = samples.sampleStackWithVESC(mock_edgedevice)
        withoutvesc = samples.sampleStackWithoutVESC(mock_edgedevice)
        common, difference, added = withoutvesc.compare(withvesc)
        
        self.assertIsNotNone(common)
        self.assertIsNotNone(difference)
        self.assertIsNotNone(added)

        self.assertEqual(len(common), 6,
                         'Number of common nodes must be 6 but it was {}'.format(len(common)))
        self.assertEqual(len(added), 15,
                         'Number of nodes to add must be 15 but it was {}'.format(len(added)))
        self.assertEqual(len(difference), 0,
                         'Number of nodes to remove must be 0 but it was {}'.format(len(difference)))

    @patch('core.model.edge_device.EdgeDevice')
    def test_stack_merge(self, mock_edgedevice):

        samples = TestSample()
        withvesc = samples.sampleStackWithVESC(mock_edgedevice)
        withoutvesc = samples.sampleStackWithoutVESC(mock_edgedevice)

        all = withoutvesc.merge(withvesc)
        
        self.assertIsNotNone(all)
      

        self.assertEqual(len(all), 21,
                         'Number of common nodes must be 6 but it was {}'.format(len(all)))

        count = 0
        for n in all:
            if n.action == 'none':
                count = count + 1
        self.assertEqual(count, 6,
                         'Number of no action must be 6 but it was {}'.format(count))
        count = 0
        for n in all:
            if n.action == 'start':
                count = count + 1
        self.assertEqual(count, 15,
                         'Number of no action must be 15 but it was {}'.format(count))
        count = 0
        for n in all:
            if n.action == 'stop':
                count = count + 1
        self.assertEqual(count, 0,
                         'Number of no action must be 0 but it was {}'.format(count))


    @patch('core.ditto.twin.Twin')
    @patch('core.model.stack.Stack')
   
    def test_edge_device(self, mock_stack, mock_twin):

        edge = edge_device.EdgeDevice(mock_twin, params={})
        edge.connect()

        mock_twin.register.assert_called_with()
        self.assertIsNotNone(edge)

if __name__ == '__main__':
    import rostest

    rostest.rosrun(
        PKG,
        'test_muto_stack_model_arg',
        CaseA)