#
#  Copyright (c) 2023 Composiv.ai
#
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# and Eclipse Distribution License v1.0 which accompany this distribution.
#
# Licensed under the  Eclipse Public License v2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# The Eclipse Public License is available at
#    http://www.eclipse.org/legal/epl-v20.html
# and the Eclipse Distribution License is available at
#   http://www.eclipse.org/org/documents/edl-v10.php.
#
# Contributors:
#    Composiv.ai - initial API and implementation
#
#

from muto_msgs.srv import CoreTwin

import json


class TwinServices():
    """ # TODO add docs."""

    def __init__(self, node, nname):
        self.node = node
        self.nname = nname

        self.node.create_service(CoreTwin, f"{self.nname}/get_current_properties", self.callback_get_current_properties)
        self.node.create_service(CoreTwin, f"{self.nname}/get_stack_definition", self.callback_get_stack_definition)
        self.node.create_service(CoreTwin, f"{self.nname}/set_current_stack", self.callback_set_current_stack)
        self.node.create_service(CoreTwin, f"{self.nname}/get_context", self.callback_get_context)
        self.node.create_service(CoreTwin, f"{self.nname}/register_device", self.callback_register_device)
        self.node.create_service(CoreTwin, f"{self.nname}/get_registered_telemetries", self.callback_get_registered_telemetries)
        self.node.create_service(CoreTwin, f"{self.nname}/register_telemetry", self.callback_register_telemetry)
        self.node.create_service(CoreTwin, f"{self.nname}/delete_telemetry", self.callback_delete_telemetry)
        self.node.create_service(CoreTwin, f"{self.nname}/get_internet_status", self.callback_get_internet_status)

    def callback_get_current_properties(self, request, response):
        """
        Callback function to handle requests for the current properties of the device's stack.

        This method is used as a ROS service callback to provide the current properties of the device's stack.
        It retrieves the properties by calling the `get_current_properties` method on the node, converts the properties
        to a JSON string, and sets this JSON string as the response output.

        Args:
            request: The service request object. It is not used in this method.
            response: The service response object. The `output` attribute of this object will be set to the JSON string 
                    representing the current stack properties.

        Returns:
            response: The modified service response object with the `output` attribute set to the JSON string of the current stack properties.
        """
        stack = self.node.get_current_properties()
        
        response.output = json.dumps(stack)

        return response
    
    def callback_get_stack_definition(self, request, response):
        """
        Service callback to get the stack definition for a given stack ID.

        This method is a ROS 2 service callback that handles requests to retrieve the stack definition.
        It extracts the stack ID from the request, retrieves the stack definition using the node's `get_stack_definition` method,
        and sets the definition in the response.

        Args:
            request: The request object containing the input stack ID.
            response: The response object to be populated with the stack definition.

        Returns:
            response: The response object with the output set to the JSON-encoded stack definition.
        """
        stack_id = request.input
        definition = self.node.get_stack_definition(stack_id)
        
        response.output = json.dumps(definition)

        return response
    
    def callback_set_current_stack(self, request, response):
        """
        Handles the service request to set the current stack of the device.

        This callback method processes a service request to update the current stack configuration of the device.
        It extracts the payload from the request, calls the `set_current_stack` method of the node to perform the update, 
        and sets the status code of the operation in the response.

        Args:
            request: The service request containing the input payload.
            response: The service response that will contain the status code of the operation.

        Returns:
            response: The response with the status code as a string in the output attribute.
        """
        payload = request.input
        status_code = self.node.set_current_stack(payload)

        response.output = str(status_code)

        return response
    
    def callback_get_context(self, request, response):
        """
        Callback method to handle requests for the device context.

        This method is invoked when a service request for the device context is received. It retrieves the context
        information of the device using the `get_context` method of the node, serializes it into a JSON string,
        and assigns it to the response's output field.

        Args:
            request: The service request object (not used in this method).
            response: The service response object that will contain the context information.

        Returns:
            response: The service response object with the context information in the output field.
        """
        context = self.node.get_context()
        
        response.output = json.dumps(context)

        return response

    def callback_register_device(self, request, response):
        """
        Registers the device with the twin server.

        This method is invoked as a callback to handle a request for registering the device with the twin server.
        It calls the 'register_device' method of the associated 'node' instance to perform the registration process.
        The status code returned by the 'register_device' method is converted to a string and set as the output in the response object.

        Args:
            request: The request object containing input data.
            response: The response object for returning output data.

        Returns:
            response: The response object containing the status code of the device registration attempt.
        """
        status_code = self.node.register_device()

        response.output = str(status_code)

        return response
    
    def callback_get_registered_telemetries(self, request, response):
        """
        Retrieves registered telemetry properties from the twin server.

        This method is a callback function designed to handle a request for retrieving registered telemetry properties from the twin server.
        It invokes the 'get_telemetry' method of the associated 'node' instance to fetch the telemetry data.
        The telemetry data is then assigned to the 'output' attribute of the response object, which is returned.

        Args:
            request: The request object containing input data.
            response: The response object for returning output data.

        Returns:
            response: The response object containing the registered telemetry properties.
        """
        telemetry_data = self.node.get_telemetry()

        response.output = telemetry_data

        return response
    
    def callback_register_telemetry(self, request, response):
        """
        Registers telemetry with the twin server.

        This method is a callback function designed to handle a request for registering telemetry with the twin server.
        It retrieves the payload from the input attribute of the request object.
        The payload is then passed to the 'register_telemetry' method of the associated 'node' instance to perform the registration.
        The status code returned by the 'register_telemetry' method is converted to a string and set as the output in the response object.

        Args:
            request: The request object containing input data.
            response: The response object for returning output data.

        Returns:
            response: The response object containing the status code of the telemetry registration attempt.
        """
        payload = request.input
        status_code = self.node.register_telemetry(payload)

        response.output = str(status_code)

        return response

    def callback_delete_telemetry(self, request, response):
        """
        Deletes telemetry from the twin server.

        This method is a callback function designed to handle a request for deleting telemetry from the twin server.
        It retrieves the payload from the input attribute of the request object.
        The payload is then passed to the 'delete_telemetry' method of the associated 'node' instance to perform the deletion.
        The status code returned by the 'delete_telemetry' method is converted to a string and set as the output in the response object.

        Args:
            request: The request object containing input data.
            response: The response object for returning output data.

        Returns:
            response: The response object containing the status code of the telemetry deletion attempt.
        """
        payload = request.input
        status_code = self.node.delete_telemetry(payload)

        response.output = str(status_code)

        return response
    
    def callback_get_internet_status(self, request, response):
        """
        Retrieves the internet connection status.

        This method is a callback function designed to handle a request for retrieving the internet connection status.
        It retrieves the internet status from the associated 'node' instance and converts it to a string.
        The string representation of the internet status is set as the output in the response object, which is returned.

        Args:
            request: The request object containing input data.
            response: The response object for returning output data.

        Returns:
            response: The response object containing the internet connection status.
        """
        response.output = str(self.node.internet_status)

        return response
        