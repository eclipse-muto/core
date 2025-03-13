#
#  Copyright (c) 2025 Composiv.ai
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

import rclpy
from rclpy.node import Node

from core.twin_services import TwinServices

import uuid
import json
import requests
import socket

class Twin(Node):
    """
    Represents a twin device node that interacts with a twin server.

    This class handles the registration, telemetry management, and status checks for a twin device.
    It extends the ROS 2 Node class and utilizes parameters to configure the device's settings and interactions
    with the twin server.

    Attributes:
        twin_url (str): The URL of the twin server.
        anonymous (bool): Whether the device should be named randomly.
        namespace (str): The namespace of the device.
        name (str): The name of the device.
        type (str): The type of the device.
        unique_name (str): The unique name of the device, randomly generated if anonymous is True.
        attributes (dict): The attributes of the device.
        definition (str): The definition of the device.
        topic (str): MQTT topic of the device.
        thing_id (str): Ditto Thing ID of the device.
        internet_status (bool): The current internet connection status.
        is_device_registered (bool): The registration status of the device.

    Methods:
        __init__(): Initializes the Twin node and sets up parameters and services.
        get_current_properties(): Retrieves the current properties of the device's stack.
        get_stack_definition(stack_id): Retrieves the stack definition for a given stack ID.
        stack(thing_id): Retrieves the stack properties for a given thing ID.
        set_current_stack(stack, state='unknown'): Sets the current stack state.
        get_context(): Returns information about the device.
        register_device(): Registers the device with the twin server.
        get_registered_telemetries(): Retrieves the registered telemetry properties from the twin server.
        register_telemetry(telemetry_to_register): Registers or updates a telemetry entry with the twin server.
        delete_telemetry(telemetry_to_delete): Deletes a telemetry entry from the twin server.
        device_register_data(): Constructs the data dictionary required for device registration with the twin server.
        connection_status(): Checks the connection status to the twin server and updates the internet status.
    """

    def __init__(self):
        super().__init__("core_twin")

        # Declare Parameters
        self.declare_parameter("twin_url", "")
        self.declare_parameter("anonymous", False)
        self.declare_parameter("namespace", "")
        self.declare_parameter("name", "")
        self.declare_parameter("type", "")
        self.declare_parameter("unique_name", "")
        self.declare_parameter("attributes", json.dumps({}))
        self.declare_parameter("definition", "org.eclipse.muto:EdgeDevice:0.0.1")
        self.declare_parameter("topic", "")
        self.declare_parameter("thing_id", "")

        # Initialize Parameters
        self.twin_url = self.get_parameter("twin_url").value
        self.anonymous = self.get_parameter("anonymous").value
        self.namespace = self.get_parameter("namespace").value
        self.name = self.get_parameter("name").value
        self.type = self.get_parameter("type").value
        self.unique_name = f"{self.type}.{str(uuid.uuid4())}" if self.anonymous else self.name
        self.attributes = json.loads(self.get_parameter("attributes").value)
        self.definition = self.get_parameter("definition").value
        self.topic = f"{self.namespace}:{self.unique_name}"
        self.thing_id = f"{self.namespace}:{self.name}"

        self.internet_status = False
        self.is_device_registered = False

        # Services
        TwinServices(self, self.get_name())

        # Internet connectivity
        socket.setdefaulttimeout(3.0)
        self.check_internet_conn = self.create_timer(3.0, self.connection_status)

        # Register Device
        if self.internet_status:
            self.register_device()

    def get_current_properties(self):
        """Retrieves the current properties of the device's stack."""
        return self.stack(self.thing_id)

    def get_stack_definition(self, stack_id: str):
        """Retrieves the stack definition for a given stack ID."""
        return self.stack(stack_id)

    def stack(self, thing_id: str) -> dict | None:
        """The method that sends the requests if  acquire stack data"""
        try:
            if (not self.twin_url) or (not thing_id):
                return None
            r = requests.get(url=self.twin_url + "/api/2/things/" +
                             thing_id + "/features/stack")
            self.get_logger().info(f"Stack getting Status Code: {r.status_code}")

            if r.status_code >= 300:
                return {}
            payload = json.loads(r.text)
            return payload.get("properties", {})

        except requests.exceptions.Timeout as t:
            self.get_logger().error(
                f"Request timed out when trying to get stack from twins repo: {t}"
            )
        except requests.exceptions.RequestException as r:
            self.get_logger().error(
                f"Request error when trying to get stack from twins repo: {r}"
            )
        except json.JSONDecodeError as j:
            self.get_logger().error(
                f"Failed to decode JSON response when getting stack from twins repo: {j}"
            )
        except Exception as e:
            self.get_logger().error(
                f"Unexpected error when trying to get stack from twins repo: {e}"
            )
        return None

    def set_current_stack(self, stack_id: str, state: str = "unknown"):
        """Sets the Edge Device's current stack to the given stack_id"""
        if not stack_id:
            return

        headers = {'Content-type': 'application/json'}

        if stack_id:
            r = requests.put(self.twin_url + "/api/2/things/{}/features/stack/properties/current".format(self.thing_id),
                             headers=headers, json={"stackId": stack_id, "state": state})
            self.get_logger().info(f"Status Code: {r.status_code}, Response: {r.text}")


    def get_context(self):
        """Return information about the device."""
        context = {}

        context["namespace"] = self.namespace
        context["topic"] = self.topic
        context["twin_url"] = self.twin_url
        context["type"] = self.type
        context["unique_name"] = self.unique_name
        context["thing_id"] = self.thing_id
        context["anonymous"] = self.anonymous

        return context

    def register_device(self):
        """
        Registers the device with the twin server.

        This method attempts to register the device by sending a PATCH request to the twin server's API.
        If the PATCH request results in a 400 status code, it attempts a PUT request with additional data.
        If the PATCH request results in a 404 status code, it attempts a PUT request with the initial data.
        The method logs the success or failure of the registration process and updates the device's registration status.

        Returns:
            int: The HTTP status code from the final registration attempt.
        """
        res = requests.patch(
            f"{self.twin_url}/api/2/things/{self.thing_id}",
            headers={"Content-type": "application/merge-patch+json"},
            json=self.device_register_data()
        )

        if res.status_code == 400:
            data = self.device_register_data()
            data['policyId'] = self.thing_id
            res = requests.put(
                f"{self.twin_url}/api/2/things/{self.thing_id}",
                headers={"Content-type": "application/json"},
                json=data
            )
        
        if res.status_code == 404:
            data = self.device_register_data()
            res = requests.put(
                f"{self.twin_url}/api/2/things/{self.thing_id}",
                headers={"Content-type": "application/json"},
                json=data
            )

        if res.status_code == 201 or res.status_code == 204:
            self.get_logger().info(f"Device registered successfully.")
            self.is_device_registered = True
        else:
            self.get_logger().warn(
                f"Device registration was unsuccessful. Status Code: {res.status_code}.")

        return res.status_code

    def get_registered_telemetries(self):
        """
        Retrieves the registered telemetry properties from the twin server.

        This method sends a GET request to the twin server's API to fetch the telemetry properties for the specified device.
        If the request is successful (HTTP status code 200), it logs an informational message.
        Otherwise, it logs a warning message with the status code and response text.
        The method returns the telemetry properties as a dictionary.

        Returns:
            dict: The telemetry properties received from the twin server.
        """
        res = requests.get(
            f"{self.twin_url}/api/2/things/{self.thing_id}/features/telemetry/properties",
            headers={"Content-type": "application/json"}
        )

        if res.status_code == 200:
            self.get_logger().info(f"Telemetry properties received successfully.")
        else:
            self.get_logger().warn(
                f"Getting telemetry properties was unsuccessful - {res.status_code} {res.text}.")

        payload = json.loads(res.text)

        return payload

    def register_telemetry(self, telemetry_to_register):
        """
        Registers or updates a telemetry entry with the twin server.

        This method takes a telemetry entry in JSON format, retrieves the current telemetry definitions from the twin server,
        and updates the list of telemetry definitions to include the new entry. It sends a PUT request to the twin server's API
        with the updated list. The method logs the success or failure of the telemetry registration or update process and returns
        the HTTP status code of the PUT request.

        Args:
            telemetry_to_register (str): A JSON string representing the telemetry entry to be registered or updated.

        Returns:
            int: The HTTP status code from the telemetry registration or update attempt.
        """
        req_telemetry = json.loads(telemetry_to_register)

        registered_telemetries = self.get_registered_telemetries()
        current_definition = registered_telemetries.get("definition", None)

        new_definition = []

        if current_definition != None:
            filtered = filter(
                lambda x: True if x.get(
                    "topic") != req_telemetry.get("topic") else False,
                current_definition
            )
            new_definition = list(filtered)

        new_definition.append(req_telemetry)

        res = requests.put(
            f"{self.twin_url}/api/2/things/{self.thing_id}/features/telemetry/properties/definition",
            headers={"Content-type": "application/json"},
            json=new_definition
        )

        if res.status_code == 201:
            self.get_logger().info(f"Telemetry registered successfully.")
        elif res.status_code == 204:
            self.get_logger().info(f"Telemetry modified successfully.")
        else:
            self.get_logger().warn(
                f"Telemetry registration was unsuccessful - {res.status_code}.")

        return res.status_code

    def delete_telemetry(self, telemetry_to_delete):
        """
        Deletes a telemetry entry from the twin server.

        This method takes a telemetry entry in JSON format, retrieves the current telemetry definitions from the twin server,
        and updates the list of telemetry definitions to remove the specified entry. It sends a PUT request to the twin server's API
        with the updated list. The method logs the success or failure of the telemetry deletion process and returns the HTTP status code
        of the PUT request.

        Args:
            telemetry_to_delete (str): A JSON string representing the telemetry entry to be deleted.

        Returns:
            int: The HTTP status code from the telemetry deletion attempt.
        """
        req_telemetry = json.loads(telemetry_to_delete)

        registered_telemetries = self.get_registered_telemetries()
        current_definition = registered_telemetries.get("definition", None)

        new_definition = []

        if current_definition != None:
            filtered = filter(
                lambda x: True if x.get(
                    "topic") != req_telemetry.get("topic") else False,
                current_definition
            )
            new_definition = list(filtered)

        res = requests.put(
            f"{self.twin_url}/api/2/things/{self.thing_id}/features/telemetry/properties/definition",
            headers={"Content-type": "application/json"},
            json=new_definition
        )

        if res.status_code == 204:
            self.get_logger().info(f"Telemetry deleted successfully.")
        else:
            self.get_logger().warn(
                f"Telemetry deletion was unsuccessful - {res.status_code}.")

        return res.status_code

    def device_register_data(self):
        """ Constructs the data dictionary required for device registration with the twin server. """
        data = {
            "definition": self.definition,
            "attributes": self.attributes,
            "features": {
                "context": {
                    "properties": {}
                },
                "stack": {
                    "properties": {}
                },
                "telemetry": {
                    "properties": {}
                }
            }
        }

        return data

    def connection_status(self):
        """
        Checks the connection status to the twin server and updates the internet status.

        This method attempts to establish a TCP connection to the twin server using a socket.
        If the connection is successful and the device is not registered, it calls the `register_device` method.
        The internet connection status is then updated accordingly. If the connection fails, it logs a warning message.
        """
        try:     
            self.socket_ = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket_.connect((self.twin_url.split("@")[1], 1883))

            if not self.is_device_registered:
                self.register_device()

            self.internet_status = True
        except socket.error as ex:
            self.internet_status = False
            self.get_logger().warn(f"Twin Server ping failed: {ex}")
        finally:
            del self.socket_

def main():
    rclpy.init()
    twin = Twin()
    rclpy.spin(twin)


if __name__ == '__main__':
    main()
