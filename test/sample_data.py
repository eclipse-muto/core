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
#! /usr/bin/env python



import core.model.node as node
import core.model.param as param
import core.model.stack as stack


class TestSample():

    def sampleStack(self, device):
        simple_stack = {
                "name": "Composiv TestCar Base Stack",
                "context": "eteration_office",
                "stackId": "ai.composiv.sandbox.f1tenth:testcar_base.launch",
                "arg": [
                    {
                        "name": "racecar_version",
                        "value": "racecar-v2"
                    },
                    {
                        "name": "sampleName",
                        "value": "echo sampleValue"
                    }]
        }
        stck =  stack.Stack(device, simple_stack, parent=None)
        return stck

    def sampleStackWithVESC(self, device):
        withvesc = stack.Stack(device, self.exampleStackA(), parent=None)
        return withvesc
    
    def sampleStackWithoutVESC(self, device):
        withoutvesc = stack.Stack(device, self.exampleStackB(), parent=None)
        return withoutvesc

    def sampleNode(self, device):
        stck = self.sampleStack(device)
        n = node.Node(stack=stck, manifest={
                    "name": "Particle_filter",
                    "pkg": "particle_filter",
                    "exec": "particle_filter.py",
                    "output": "screen",
                    "param": [
                        {
                            "name": "scan_topic",
                            "value": "/scan"
                        }, 
                        {
                            "name": "odometry_topic",
                            "value": "/vesc/odom"
                        },
                        {
                            "name": "angle_step",
                            "value": "18"
                        },
                        {
                            "name": "max_particles",
                            "value": "4000"
                        },
                        {
                            "name": "max_viz_particles",
                            "value": "60"
                        },
                        {
                            "name": "range_method",
                            "value": "rmgpu"
                        },
                        {
                            "name": "squash_factor",
                            "value": "2.2"
                        },
                        {
                            "name": "theta_discretization",
                            "value": "112"
                        },
                        {
                            "name": "max_range",
                            "value": "10"
                        },
                        {
                            "name": "viz",
                            "value": "1"
                        },
                        {
                            "name": "fine_timing",
                            "value": "0"
                        },
                        {
                            "name": "publish_odom",
                            "value": "1"
                        },
                        {
                            "name": "z_short",
                            "value": "0.01"
                        },
                        {
                            "name": "z_max",
                            "value": "0.07"
                        },
                        {
                            "name": "z_rand",
                            "value": "0.12"
                        },
                        {
                            "name": "z_hit",
                            "value": "0.75"
                        },
                        {
                            "name": "sigma_hit",
                            "value": "8.0"
                        },
                        {
                            "name": "motion_dispersion_x",
                            "value": "0.05"
                        },
                        {
                            "name": "motion_dispersion_y",
                            "value": "0.025"
                        },
                        {
                            "name": "motion_dispersion_theta",
                            "value": "0.25"
                        },
                        {
                            "name": "rangelib_variant",
                            "value": "2"
                        }
                    ]
        })
        return n
    def sampleNodeWithArgParam(self, device):
        stck = self.sampleStack(device)
        n = node.Node(stack=stck, manifest={
                    "name": "Particle_filter",
                    "pkg": "particle_filter",
                    "exec": "particle_filter.py",
                    "output": "screen",
                    "param": [
                        {
                            "name": "scan_topic",
                            "value": "/scan"
                        }, {
                            "name": "test_param",
                            "value": "$(arg racecar_version)"
                        },
                        {
                            "name": "odometry_topic",
                            "value": "/vesc/odom"
                        },
                        {
                            "name": "angle_step",
                            "value": "18"
                        },
                        {
                            "name": "max_particles",
                            "value": "4000"
                        },
                        {
                            "name": "max_viz_particles",
                            "value": "60"
                        },
                        {
                            "name": "range_method",
                            "value": "rmgpu"
                        },
                        {
                            "name": "squash_factor",
                            "value": "2.2"
                        },
                        {
                            "name": "theta_discretization",
                            "value": "112"
                        },
                        {
                            "name": "max_range",
                            "value": "10"
                        },
                        {
                            "name": "viz",
                            "value": "1"
                        },
                        {
                            "name": "fine_timing",
                            "value": "0"
                        },
                        {
                            "name": "publish_odom",
                            "value": "1"
                        },
                        {
                            "name": "z_short",
                            "value": "0.01"
                        },
                        {
                            "name": "z_max",
                            "value": "0.07"
                        },
                        {
                            "name": "z_rand",
                            "value": "0.12"
                        },
                        {
                            "name": "z_hit",
                            "value": "0.75"
                        },
                        {
                            "name": "sigma_hit",
                            "value": "8.0"
                        },
                        {
                            "name": "motion_dispersion_x",
                            "value": "0.05"
                        },
                        {
                            "name": "motion_dispersion_y",
                            "value": "0.025"
                        },
                        {
                            "name": "motion_dispersion_theta",
                            "value": "0.25"
                        },
                        {
                            "name": "rangelib_variant",
                            "value": "2"
                        }
                    ]
        })
        return n

    def exampleStackA(self):
        stack = {
            "name": "Composiv TestCar Base Stack",
            "context": "eteration_office",
            "stackId": "ai.composiv.sandbox.f1tenth:testcar_base.launch",
            "arg": [
                {
                    "name": "racecar_version",
                    "value": "racecar-v2"
                },
                {
                    "name": "run_camera",
                    "value": "false"
                },
                {
                    "name": "testcar_ws",
                    "value": "$(find muto_core)/test"
                },
                {
                    "name": "joy_teleop_config",
                    "value": "$(arg testcar_ws)/configs/joy_teleop.yaml"
                },
                {
                    "name": "vesc_config",
                    "value": "$(arg testcar_ws)/configs/vesc.yaml"
                },
                {
                    "name": "sensors_config",
                    "value": "$(arg testcar_ws)/configs/sensors.yaml"
                },
                {
                    "name": "map",
                    "value": "$(arg testcar_ws)/maps/etr_office_1430.yaml"
                }
            ],
            "param": [
                {
                    "namespace": "vesc",
                    "from": "$(arg joy_teleop_config)"
                },
                {
                    "namespace": "vesc",
                    "from": "$(arg vesc_config)"
                },
                {
                    "from": "$(arg sensors_config)"
                },
                {
                    "name": "port",
                    "value": "/dev/ttyACM0"
                },
                {
                    "name": "nav_topic",
                    "value": "/vesc/high_level/ackermann_cmd_mux/input/nav_1"
                },
                {
                    "name": "scan_topic",
                    "value": "/scan"
                }
            ],
            "node": [
                {
                    "name": "map_server",
                    "pkg": "map_server",
                    "exec": "map_server",
                    "args": "$(arg map)"
                },
                {
                    "name": "Particle_filter",
                    "pkg": "particle_filter",
                    "exec": "particle_filter.py",
                    "output": "screen",
                    "param": [
                        {
                            "name": "scan_topic",
                            "value": "/scan"
                        },
                        {
                            "name": "odometry_topic",
                            "value": "/vesc/odom"
                        },
                        {
                            "name": "angle_step",
                            "value": "18"
                        },
                        {
                            "name": "max_particles",
                            "value": "4000"
                        },
                        {
                            "name": "max_viz_particles",
                            "value": "60"
                        },
                        {
                            "name": "range_method",
                            "value": "rmgpu"
                        },
                        {
                            "name": "squash_factor",
                            "value": "2.2"
                        },
                        {
                            "name": "theta_discretization",
                            "value": "112"
                        },
                        {
                            "name": "max_range",
                            "value": "10"
                        },
                        {
                            "name": "viz",
                            "value": "1"
                        },
                        {
                            "name": "fine_timing",
                            "value": "0"
                        },
                        {
                            "name": "publish_odom",
                            "value": "1"
                        },
                        {
                            "name": "z_short",
                            "value": "0.01"
                        },
                        {
                            "name": "z_max",
                            "value": "0.07"
                        },
                        {
                            "name": "z_rand",
                            "value": "0.12"
                        },
                        {
                            "name": "z_hit",
                            "value": "0.75"
                        },
                        {
                            "name": "sigma_hit",
                            "value": "8.0"
                        },
                        {
                            "name": "motion_dispersion_x",
                            "value": "0.05"
                        },
                        {
                            "name": "motion_dispersion_y",
                            "value": "0.025"
                        },
                        {
                            "name": "motion_dispersion_theta",
                            "value": "0.25"
                        },
                        {
                            "name": "rangelib_variant",
                            "value": "2"
                        }
                    ]
                },
                {
                    "namespace": "vesc",
                    "name": "joy_node",
                    "pkg": "joy",
                    "exec": "joy_node"
                },
                {
                    "namespace": "vesc",
                    "name": "joy_teleop",
                    "pkg": "racecar",
                    "exec": "joy_teleop.py"
                },
                {
                    "namespace": "vesc",
                    "name": "mux_chainer",
                    "pkg": "topic_tools",
                    "exec": "relay",
                    "args": "/vesc/high_level/ackermann_cmd_mux/output /vesc/low_level/ackermann_cmd_mux/input/navigation"
                },
                {
                    "namespace": "vesc",
                    "name": "mux_topic_backward_compat_safety",
                    "pkg": "topic_tools",
                    "exec": "relay",
                    "args": "/vesc/ackermann_cmd_mux/input/safety /vesc/low_level/ackermann_cmd_mux/input/safety"
                },
                {
                    "namespace": "vesc",
                    "name": "mux_topic_backward_compat_teleop",
                    "pkg": "topic_tools",
                    "exec": "relay",
                    "args": "/vesc/ackermann_cmd_mux/input/teleop /vesc/low_level/ackermann_cmd_mux/input/teleop"
                },
                {
                    "namespace": "vesc",
                    "name": "mux_topic_backward_compat_navigation",
                    "pkg": "topic_tools",
                    "exec": "relay",
                    "args": "/vesc/ackermann_cmd_mux/input/navigation /vesc/high_level/ackermann_cmd_mux/input/nav_0"
                },
                {
                    "namespace": "vesc",
                    "name": "zero_ackermann_cmd",
                    "pkg": "rostopic",
                    "exec": "rostopic",
                    "args": "pub -r 6 high_level/ackermann_cmd_mux/input/default ackermann_msgs/AckermannDriveStamped '{header: auto, drive: { steering_angle: 0.0, speed: 0.0} }'"
                },
                {
                    "namespace": "vesc/high_level",
                    "name": "ackermann_cmd_mux_nodelet_manager",
                    "pkg": "nodelet",
                    "exec": "nodelet",
                    "args": "manager"
                },
                {
                    "namespace": "vesc/high_level",
                    "name": "ackermann_cmd_mux",
                    "pkg": "nodelet",
                    "exec": "nodelet",
                    "args": "load ackermann_cmd_mux/AckermannCmdMuxNodelet ackermann_cmd_mux_nodelet_manager",
                    "param": [
                        {
                            "name": "yaml_cfg_file",
                            "value": "$(arg testcar_ws)/configs/high_level_mux.yaml"
                        }
                    ]
                },
                {
                    "namespace": "vesc/low_level",
                    "name": "ackermann_cmd_mux_nodelet_manager",
                    "pkg": "nodelet",
                    "exec": "nodelet",
                    "args": "manager"
                },
                {
                    "namespace": "vesc/low_level",
                    "name": "ackermann_cmd_mux",
                    "pkg": "nodelet",
                    "exec": "nodelet",
                    "args": "load ackermann_cmd_mux/AckermannCmdMuxNodelet ackermann_cmd_mux_nodelet_manager",
                    "param": [
                        {
                            "name": "yaml_cfg_file",
                            "value": "$(arg testcar_ws)/configs/low_level_mux.yaml"
                        }
                    ]
                },
                {
                    "namespace": "vesc",
                    "pkg": "vesc_ackermann",
                    "exec": "ackermann_to_vesc_node",
                    "name": "ackermann_to_vesc",
                    "remap": [
                        {
                            "from": "ackermann_cmd",
                            "to": "low_level/ackermann_cmd_mux/output"
                        },
                        {
                            "from": "commands/motor/speed",
                            "to": "commands/motor/unsmoothed_speed"
                        },
                        {
                            "from": "commands/servo/position",
                            "to": "commands/servo/unsmoothed_position"
                        }
                    ]
                },
                {
                    "namespace": "vesc",
                    "pkg": "vesc_driver",
                    "exec": "vesc_driver_node",
                    "name": "vesc_driver"
                },
                {
                    "namespace": "vesc",
                    "pkg": "vesc_ackermann",
                    "exec": "vesc_to_odom_node",
                    "name": "vesc_to_odom"
                },
                {
                    "namespace": "vesc",
                    "name": "throttle_interpolator",
                    "pkg": "ackermann_cmd_mux",
                    "exec": "throttle_interpolator.py"
                },
                {
                    "name": "laser_node",
                    "pkg": "urg_node",
                    "exec": "urg_node"
                },
                {
                    "name": "base_link_to_imu",
                    "pkg": "tf2_ros",
                    "exec": "static_transform_publisher",
                    "args": "0.245 0.0 0.117    0.7071067811865475 0.7071067811865475 0.0 0.0 /base_link /base_imu_link"
                },
                {
                    "name": "base_link_to_laser",
                    "pkg": "tf2_ros",
                    "exec": "static_transform_publisher",
                    "args": "0.285 0.0 0.0 0.0 0.0 0.0 1.0 /base_link /laser"
                },
                {
                    "name": "base_link_to_base_footprint",
                    "pkg": "tf2_ros",
                    "exec": "static_transform_publisher",
                    "args": "0.0 0.0 0.0     0.0 0.0 0.0 1.0 /base_link /base_footprint"
                }
            ]
        }

        return stack

    def exampleStackB(self):
        stack = {
            "name": "Composiv TestCar Base Stack with no VESC",
            "context": "eteration_office",
            "stackId": "ai.composiv.sandbox.f1tenth:testcar_base_novesc.launch",
            "arg": [
                {
                    "name": "racecar_version",
                    "value": "racecar-v2"
                },
                {
                    "name": "run_camera",
                    "value": "false"
                },
                {
                    "name": "testcar_ws",
                    "value": "$(find muto_core)/test"
                },
                {
                    "name": "joy_teleop_config",
                    "value": "$(arg testcar_ws)/configs/joy_teleop.yaml"
                },
                {
                    "name": "sensors_config",
                    "value": "$(arg testcar_ws)/configs/sensors.yaml"
                },
                {
                    "name": "map",
                    "value": "$(arg testcar_ws)/maps/etr_office_1430.yaml"
                }
            ],
            "param": [
                {
                    "from": "$(arg sensors_config)"
                },
                {
                    "name": "port",
                    "value": "/dev/ttyACM0"
                },
                {
                    "name": "nav_topic",
                    "value": "/vesc/high_level/ackermann_cmd_mux/input/nav_1"
                },
                {
                    "name": "scan_topic",
                    "value": "/scan"
                }
            ],
            "node": [
                {
                    "name": "map_server",
                    "pkg": "map_server",
                    "exec": "map_server",
                    "args": "$(arg map)"
                },
                {
                    "name": "Particle_filter",
                    "pkg": "particle_filter",
                    "exec": "particle_filter.py",
                    "output": "screen",
                    "param": [
                        {
                            "name": "scan_topic",
                            "value": "/scan"
                        },
                        {
                            "name": "odometry_topic",
                            "value": "/vesc/odom"
                        },
                        {
                            "name": "angle_step",
                            "value": "18"
                        },
                        {
                            "name": "max_particles",
                            "value": "4000"
                        },
                        {
                            "name": "max_viz_particles",
                            "value": "60"
                        },
                        {
                            "name": "range_method",
                            "value": "rmgpu"
                        },
                        {
                            "name": "squash_factor",
                            "value": "2.2"
                        },
                        {
                            "name": "theta_discretization",
                            "value": "112"
                        },
                        {
                            "name": "max_range",
                            "value": "10"
                        },
                        {
                            "name": "viz",
                            "value": "1"
                        },
                        {
                            "name": "fine_timing",
                            "value": "0"
                        },
                        {
                            "name": "publish_odom",
                            "value": "1"
                        },
                        {
                            "name": "z_short",
                            "value": "0.01"
                        },
                        {
                            "name": "z_max",
                            "value": "0.07"
                        },
                        {
                            "name": "z_rand",
                            "value": "0.12"
                        },
                        {
                            "name": "z_hit",
                            "value": "0.75"
                        },
                        {
                            "name": "sigma_hit",
                            "value": "8.0"
                        },
                        {
                            "name": "motion_dispersion_x",
                            "value": "0.05"
                        },
                        {
                            "name": "motion_dispersion_y",
                            "value": "0.025"
                        },
                        {
                            "name": "motion_dispersion_theta",
                            "value": "0.25"
                        },
                        {
                            "name": "rangelib_variant",
                            "value": "2"
                        }
                    ]
                },
                {
                    "name": "laser_node",
                    "pkg": "urg_node",
                    "exec": "urg_node"
                },
                {
                    "name": "base_link_to_imu",
                    "pkg": "tf2_ros",
                    "exec": "static_transform_publisher",
                    "args": "0.245 0.0 0.117    0.7071067811865475 0.7071067811865475 0.0 0.0 /base_link /base_imu_link"
                },
                {
                    "name": "base_link_to_laser",
                    "pkg": "tf2_ros",
                    "exec": "static_transform_publisher",
                    "args": "0.285 0.0 0.0 0.0 0.0 0.0 1.0 /base_link /laser"
                },
                {
                    "name": "base_link_to_base_footprint",
                    "pkg": "tf2_ros",
                    "exec": "static_transform_publisher",
                    "args": "0.0 0.0 0.0     0.0 0.0 0.0 1.0 /base_link /base_footprint"
                }
            ]
        }
        return stack

