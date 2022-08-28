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

from os import uname
from re import S
import unittest

from unittest.mock import patch
import rospy
import core.model.param as param
from sample_data import TestSample

PKG = 'muto_core'
NODE_NAME = 'muto_core'


_from_file = '$(find muto_core)/test/configs/amcl.yaml'


_manifest_LSFF = {'from' :_from_file}


class CaseC(unittest.TestCase):

    @patch('core.model.edge_device.EdgeDevice')
    def setUp(self,mock_device):
        rospy.init_node(NODE_NAME)
        samples = TestSample()
        self.stack = samples.sampleStack(mock_device)
        self.paramObj = param.Param(self.stack,_manifest_LSFF)
        self.paramObj.load()
        self.assertIsNotNone(self.paramObj)

    def test_param_wFF(self):
        #load ile param server'a file aktarıldı bu sebeple artık init test edilecek. ilk if kosulu olan from file uzerınden init test
        #ornek degerler
        #z_hit : 0.5
        #odom_frame_id : laser
        manifest = {'name':rospy.get_param('/odom_frame_id'),'value':rospy.get_param('/laser_z_hit'),'sep':rospy.get_param('/odom_frame_id'),
        'param':rospy.get_param_names(),'namespace':rospy.get_param('/odom_frame_id'),'from':self.paramObj.from_file}

        paramObj = param.Param(self.stack,manifest=manifest)

        self.assertEqual(
            paramObj.name,manifest['name'], 'param name was {}'.format(manifest['name']))
        self.assertEqual(
           paramObj.value,0.5, 'param value was {}'.format(manifest['value']))
        self.assertEqual(
            paramObj.sep, manifest['sep'], 'param sep was {}'.format(manifest['sep']))
        self.assertEqual(
            paramObj.from_file,self.stack.resolveExpression(manifest['from']), 'param from_file was {}'.format(self.stack.resolveExpression(manifest['from'])))
        self.assertEqual(
           paramObj.namespace, manifest['namespace'], 'param namespace was {}'.format(manifest['namespace']))
        self.assertEqual(
            paramObj._command,'', 'param command was {}'.format(''))
    
    def test_param_wC(self):
        
        #init elif command test (testing with real world data : value == 'http://localhost:11311')
        manifest = {'name':rospy.get_param('/odom_frame_id'),'value':rospy.get_param('/odom_frame_id'),'sep':rospy.get_param('/odom_frame_id'),
        'param':rospy.get_param_names(),'namespace':rospy.get_param('/odom_frame_id'),'command':'$(env ROS_MASTER_URI)'}

        paramObj = param.Param(self.stack,manifest=manifest)

        self.assertEqual(
            paramObj.name,manifest['name'], 'param name was {}'.format(manifest['name']))
        self.assertEqual(
           paramObj.value,self.stack.resolveExpression(manifest['command']), 'param value was {}'.format(self.stack.resolveExpression(manifest['command'])))
        self.assertEqual(
            paramObj.sep, manifest['sep'], 'param sep was {}'.format(manifest['sep']))
        self.assertEqual(
            paramObj.from_file,'', 'param from_file was {}'.format(''))
        self.assertEqual(
           paramObj.namespace, manifest['namespace'], 'param namespace was {}'.format(manifest['namespace']))
        self.assertEqual(
            paramObj._command,manifest['command'], 'param command was {}'.format(manifest['command'],))
            
    def test_param_wV(self):
        
        #init else value test (testing with real world data : value == 'http://localhost:11311')
        manifest = {'name':rospy.get_param('/odom_frame_id'),'value':'$(env ROS_MASTER_URI)','sep':rospy.get_param('/odom_frame_id'),
        'param':rospy.get_param_names(),'namespace':rospy.get_param('/odom_frame_id')}

        paramObj = param.Param(self.stack,manifest=manifest)

        self.assertEqual(
            paramObj.name,manifest['name'], 'param name was {}'.format(manifest['name']))
        self.assertEqual(
           paramObj.value,self.stack.resolveExpression(manifest['value']), 'param value was {}'.format(self.stack.resolveExpression(manifest['value'])))
        self.assertEqual(
            paramObj.sep, manifest['sep'], 'param sep was {}'.format(manifest['sep']))
        self.assertEqual(
            paramObj.from_file,'', 'param from_file was {}'.format(''))
        self.assertEqual(
           paramObj.namespace, manifest['namespace'], 'param namespace was {}'.format(manifest['namespace']))
        self.assertEqual(
            paramObj._command,'', 'param command was {}'.format(''))
    
    def test_eq(self):

        manifest_new = {  'name' : 'param1Name', 'value': 'param1Val',
        'sep': 'param1Sep', 'namespace':'param1NameSpace', 'param':'param1Param', 'from' : 'param1From','command':'param1Command'}
        param1 = param.Param(self.stack,manifest=manifest_new)
        param2 = param.Param(self.stack,manifest=manifest_new)

        #None Check
        self.assertIsNotNone(param1)
        self.assertIsNotNone(param2)

        #is instance check 
        self.assertEqual(param1 == self.stack,False)
        self.assertEqual(param1 != self.stack,True)

        self.assertEqual(param1 == param2,True)
        self.assertEqual(param1 != param2,False)
    
    def test_load_C(self):
        manifest = {'name':'description','sep':'sep',
        'param':'param','namespace':'short','command':'$(arg sampleName)'}
        paramObj = param.Param(self.stack,manifest=manifest)
        paramObj.load()
        self.assertEqual(rospy.get_param(manifest['namespace'] + manifest['name']) == "sampleValue",True)
    
    def test_load_V(self):
        manifest = {'name':'URI','value':'$(env ROS_MASTER_URI)','sep':rospy.get_param('/odom_frame_id'),
        'param':rospy.get_param_names(),'namespace':'ROS_MASTER'}

        paramObj = param.Param(self.stack,manifest=manifest)
        paramObj.load()
        self.assertEqual(rospy.get_param(manifest['namespace'] + manifest['name']) == "http://localhost:11311",True)

     
    
if __name__ == '__main__':
    import rostest

    rostest.rosrun(
        PKG,
        'Test Muto Stack Model: Param',
        CaseC)