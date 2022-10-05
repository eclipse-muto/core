#!/usr/bin/env python
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


from unittest.mock import patch
import core.model.node as node
from sample_data import TestSample

PKG = 'muto_core'
NODE_NAME = 'muto_core'


_manifest = {
    "env": 'test-env',
    "launch-prefix": "test_launch_prefix",
    "namespace": "vesc",
    "pkg": "vesc_ackermann",
    "exec": "ackermann_to_vesc_node",
    "name": "ackermann_to_vesc",
    "if": "true",
    "unless": "false",
    "args": "racecar_version",
    "ros_args": "_ros_args",
    "output": "screen",
    "param": [{"name": "scan_topic","value": "/scan"}],
    "remap": [{"from": "ackermann_cmd", "to": "low_level/ackermann_cmd_mux/output"}]
}


class TestNode(unittest.TestCase):

    def setUp(self):
        rospy.init_node(NODE_NAME)
        self.manifest = _manifest

    @patch('core.model.edge_device.EdgeDevice')
    def test_param(self, mock_device):
        samples = TestSample()
        stack = samples.sampleStack(mock_device)
        testnode = node.Node(stack=stack, manifest=self.manifest)

        self.assertIsNotNone(testnode)
        self.assertEqual(
            testnode._env, _manifest['env'], 'arg name was {}'.format(_manifest['env']))
        self.assertEqual(
            len(testnode._param), len(_manifest['param']), 'arg name was {}'.format(_manifest['param']))
        self.assertEqual(
            len(testnode._remap), len(_manifest['remap']), 'arg name was {}'.format(_manifest['remap']))
        self.assertEqual(
            testnode._pkg, _manifest['pkg'], 'arg name was {}'.format(_manifest['pkg']))
        self.assertEqual(
            testnode._exec, _manifest['exec'], 'arg name was {}'.format(_manifest['exec']))
        self.assertEqual(
            testnode._name, _manifest['name'], 'arg name was {}'.format(_manifest['name']))
        
        self.assertEqual(
            testnode._ros_args, _manifest['ros_args'], 'arg name was {}'.format(_manifest['ros_args']))
        self.assertEqual(
            testnode._args, _manifest['args'], 'arg name was {}'.format(_manifest['args']))
        self.assertEqual(
            testnode._namespace, _manifest['namespace'], 'arg name was {}'.format(_manifest['namespace']))
        self.assertEqual(
            testnode._launch_prefix, _manifest['launch-prefix'], 'arg name was {}'.format(_manifest['launch-prefix']))
        self.assertEqual(
            testnode._output, _manifest['output'], 'arg name was {}'.format(_manifest['output']))
        self.assertEqual(
            testnode._iff, _manifest['if'], 'arg name was {}'.format(_manifest['if']))
        self.assertEqual(
            testnode._unless, _manifest['unless'], 'arg name was {}'.format(_manifest['unless']))


if __name__ == '__main__':
    import rostest

    rostest.rosrun(
        PKG,
        'TestNode',
        TestNode)
