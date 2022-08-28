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
#  Partially adapted from code ros.org/api/rosnode
# Software License Agreement (BSD License) 
# 
# Copyright (c) 2008, Willow Garage, Inc. 
# All rights reserved. 
# 
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions 
# are met: 
# 
#  * Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer. 
#  * Redistributions in binary form must reproduce the above 
#    copyright notice, this list of conditions and the following 
#    disclaimer in the documentation and/or other materials provided 
#    with the distribution. 
#  * Neither the name of Willow Garage, Inc. nor the names of its 
#    contributors may be used to endorse or promote products derived 
#    from this software without specific prior written permission. 
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
# POSSIBILITY OF SUCH DAMAGE. 
# 

try: 
   from xmlrpc.client import ServerProxy 
except ImportError: 
   from xmlrpclib import ServerProxy 
import socket
import rosgraph
import rosnode


ID = '/rosnode'
NAME = 'rosnode'

class Introspector(object):

    def __init__(self):
        self.master = rosgraph.Master("master")

    def get_node(self,name):

        node_name = rosgraph.names.script_resolve_name('rosnode', name) 
        node_info = self.get_node_info_description(node_name)
        allNodes = rosnode.rosnode_listnodes(list_all=True)

        node_api = rosnode.get_api_uri(self.master, node_name) 
        if not node_api: 
            print("cannot contact [%s]: unknown node"%node_name) 
            return None

        print("\ncontacting node %s ..."%name,) 
        connection_info =  self.get_node_connection_info_description(node_api, self.master) 
        node_info['connection'] = connection_info
        return node_info

    def kill(self,name):
        print("\nKilling node %s ..."%name,) 
        rosnode.kill_nodes([name])


    def get_node_info_description(self,node_name): 
          # go through the master system state first 
        try: 
            state = self.master.getSystemState() 
            pub_topics = self.master.getPublishedTopics('/') 
        except socket.error: 
            raise Exception("Unable to communicate with master!") 
        pubs = [t for t, l in state[0] if node_name in l] 
        subs = [t for t, l in state[1] if node_name in l] 
        srvs = [t for t, l in state[2] if node_name in l]   

        node_info = {
            "publishes": pubs,
            "subscribes": subs,
            "services": srvs
        }
        return node_info

    def get_node_connection_info_description(self,node_api, master): 
        #turn down timeout on socket library 
        socket.setdefaulttimeout(5.0) 
        node = ServerProxy(node_api) 
        system_state = master.getSystemState() 
        connection_info = { 'businfo': []}
        try: 
            connection_info['pid'] = self._succeed(node.getPid(ID)) 
            #master_uri = _succeed(node.getMasterUri(ID)) 
            businfo = self._succeed(node.getBusInfo(ID)) 
            if businfo: 
                for info in businfo: 
                    c = {
                      "dest_id" : info[1] ,
                    "direction" :  info[2] ,
                    "transport": info[3] ,
                    "topic":  info[4] 
                    }
                    connection_info['businfo'].append(c)
                    if len(info) > 5: 
                        c['connected'] = info[5] 
                    else: 
                        c['connected'] = True #backwards compatibility 
        except socket.error: 
            print("Communication with node[%s] failed!"%(node_api)) 
            return {}
        return connection_info 

    def _succeed(self, args): 
        code, msg, val = args 
        if code != 1: 
            raise Exception("remote call failed: %s"%msg) 
        return val 
