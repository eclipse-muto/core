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

import re
import unittest
import rospy


from unittest.mock import patch
import core.model.node as node
import core.model.stack as stack
from sample_data import TestSample
import core.model.param as param

PKG = 'muto_core'
NODE_NAME = 'muto_core'



_env = 'node_test_env'
_param = 'node_test_param'
_remap = 'node_test_remap'
_pkg = 'node_test_pkg'
_exec = 'node_test_exec'
_name = 'node_test_name'
_ros_args = 'node_test_ros_args'
_args = 'node_test_args'
_namespace = 'node_test_namespace'
_launch_prefix = 'node_test_launch_prefix'
_output = 'node_test_output'
_iff = 'node_test_iff'
_unless = 'node_test_unless'
_action = None




_manifest = {'env':_env, 'param':_param, 'remap':_remap, 'pkg':_pkg, 'exec':_exec, 'name':_name, 'ros_args':_ros_args,
'args':_args, 'namespace':_namespace, 'launch_prefix':_launch_prefix, 'output':_output, 'iff':_iff, 'unless':_unless}


class testnode(unittest.TestCase):

    def setUp(self):
        rospy.init_node(NODE_NAME)
        self.manifest = _manifest

    @patch('core.model.edge_device.EdgeDevice')
    def test_param(self,mock_device):
        samples = TestSample()
        stack = samples.sampleStack(mock_device)
        testnode = node.Node(stack=stack,manifest=self.manifest)



        self.assertIsNotNone(testnode)
        self.assertEqual(
            testnode._env, _manifest['env'], 'arg name was {}'.format(_manifest['env']))
        self.assertEqual(
            testnode._param, _manifest['param'], 'arg name was {}'.format(_manifest['param']))
        self.assertEqual(
            testnode._remap, _manifest['remap'], 'arg name was {}'.format(_manifest['remap']))
        self.assertEqual(
            testnode._pkg, _manifest['pkg'], 'arg name was {}'.format(_manifest['pkg']))
        self.assertEqual(
            testnode._exec, _manifest['exec'], 'arg name was {}'.format(_manifest['exec']))
        self.assertEqual(
            testnode._name, _manifest['name'], 'arg name was {}'.format(_manifest['name']))
        self.assertEqual(
            testnode
        )
        self.assertEqual(
            testnode._ros_args, _manifest['ros_args'], 'arg name was {}'.format(_manifest['ros_args']))
        self.assertEqual(
            testnode._args, _manifest['args'], 'arg name was {}'.format(_manifest['args']))
        self.assertEqual(
            testnode._namespace, _manifest['namespace'], 'arg name was {}'.format(_manifest['namespace']))
        self.assertEqual(
            testnode._launch_prefix, _manifest['launch_prefix'], 'arg name was {}'.format(_manifest['launch_prefix']))
        self.assertEqual(
            testnode._output, _manifest['output'], 'arg name was {}'.format(_manifest['output']))
        self.assertEqual(
            testnode._iff, _manifest['iff'], 'arg name was {}'.format(_manifest['iff']))
        self.assertEqual(
            testnode._unless, _manifest['unless'], 'arg name was {}'.format(_manifest['unless']))
        self.assertEqual(
            testnode
        )




    #def ifTest(self):
      #  rospy.init_node(NODE_NAME)
      #  self.manifest = _manifest

      #  self.assertEqual(
       #     testnode.self._param.pDef, ['namespace']
      #  )


       # self._remap_args = []
       # for rm in self._remap:
        #    fr = stack.resolveExpression(rm['from'])
       #     to = stack.resolveExpression(rm['to'])
       #     self._remap_args.append((fr,to))
        


    #def eql(self, other):
       
        #self.assertnotEqual(
       #     node.self._param.pDef, other
      #  )



if __name__ == '__main__':
    import rostest

    rostest.rosrun(
        PKG,
        'test_core_node',
        testnode)