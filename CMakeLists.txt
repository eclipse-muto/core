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
cmake_minimum_required(VERSION 3.0.2)
project(muto_core)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rostest
)


catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES muto_core
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# only while under test
if (CATKIN_ENABLE_TESTING)
    # install our Python tests
    find_package(rostest REQUIRED)

    install(
            PROGRAMS
            test/core_testcases_a.py
            test/core_testcases_b.py
            test/core_testcases_param.py
            test/core_testcases_edge_device.py
            test/core_testcase_node.py
            DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    # install our test .launch files
    install(
            FILES
            test/test.launch
            test/edge.launch
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

    # tell rostest to test some Python code (point it to a .launch file that describes the setup and execution)
    add_rostest(test/test.launch) 

endif ()