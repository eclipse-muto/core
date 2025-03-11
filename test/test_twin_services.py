import json
import unittest
from unittest.mock import MagicMock, patch

import rclpy
# from muto_msgs.srv import CoreTwin

from core.twin_services import TwinServices


class TestTwinServices(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @patch("core.twin_services.CoreTwin")
    def setUp(self, mock_core_twin):
        self.mock_node = MagicMock()

        self.mock_node.get_current_properties.return_value = {
            "property_key": "test_property_value"
        }
        self.mock_node.get_stack_definition.return_value = {
            "stack_key": "test_stack_value"
        }
        self.mock_node.set_current_stack.return_value = 200
        self.mock_node.get_context.return_value = {"context_key": "test_context_value"}
        self.mock_node.register_device.return_value = 201
        self.mock_node.get_telemetry.return_value = (
            '{"telemetry_key": "test_telemetry_value"}'
        )
        self.mock_node.register_telemetry.return_value = 202
        self.mock_node.delete_telemetry.return_value = 203
        self.mock_node.internet_status = True

        self.twin_services = TwinServices(self.mock_node, "test_node_name")

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def test_callback_get_current_properties(self):
        request = MagicMock().Request()
        response = MagicMock().Response()

        response = self.twin_services.callback_get_current_properties(request, response)

        self.mock_node.get_current_properties.assert_called_once()

        expected = json.dumps({"property_key": "test_property_value"})
        self.assertEqual(response.output, expected)

    def test_callback_get_stack_definition(self):
        request = MagicMock().Request()
        request.input = "test_stack_id"
        response = MagicMock().Response()

        response = self.twin_services.callback_get_stack_definition(request, response)

        self.mock_node.get_stack_definition.assert_called_once_with("test_stack_id")

        expected = json.dumps({"stack_key": "test_stack_value"})
        self.assertEqual(response.output, expected)

    def test_callback_set_current_stack(self):
        request = MagicMock().Request()
        request.input = '{"stack_config": "test_input_config"}'
        response = MagicMock().Response()

        response = self.twin_services.callback_set_current_stack(request, response)

        self.mock_node.set_current_stack.assert_called_once_with(
            '{"stack_config": "test_input_config"}'
        )

        self.assertEqual(response.output, "200")

    def test_callback_get_context(self):
        request = MagicMock().Request()
        response = MagicMock().Response()

        response = self.twin_services.callback_get_context(request, response)

        self.mock_node.get_context.assert_called_once()

        expected = json.dumps({"context_key": "test_context_value"})
        self.assertEqual(response.output, expected)

    def test_callback_register_device(self):
        request = MagicMock().Request()
        response = MagicMock().Response()

        response = self.twin_services.callback_register_device(request, response)

        self.mock_node.register_device.assert_called_once()

        self.assertEqual(response.output, "201")

    def test_callback_get_registered_telemetries(self):
        request = MagicMock().Request()
        response = MagicMock().Response()

        response = self.twin_services.callback_get_registered_telemetries(
            request, response
        )

        self.mock_node.get_telemetry.assert_called_once()

        self.assertEqual(response.output, '{"telemetry_key": "test_telemetry_value"}')

    def test_callback_register_telemetry(self):
        request = MagicMock().Request()
        request.input = '{"new_telemetry": "test_new_telemetry_value"}'
        response = MagicMock().Response()

        response = self.twin_services.callback_register_telemetry(request, response)

        self.mock_node.register_telemetry.assert_called_once_with(
            '{"new_telemetry": "test_new_telemetry_value"}'
        )

        self.assertEqual(response.output, "202")

    def test_callback_delete_telemetry(self):
        request = MagicMock().Request()
        request.input = '{"telemetry_to_delete": "test_delete_telemetry_value"}'
        response = MagicMock().Response()

        response = self.twin_services.callback_delete_telemetry(request, response)

        self.mock_node.delete_telemetry.assert_called_once_with(
            '{"telemetry_to_delete": "test_delete_telemetry_value"}'
        )

        self.assertEqual(response.output, "203")

    def test_callback_get_internet_status(self):
        request = MagicMock().Request()
        response = MagicMock().Response()

        response = self.twin_services.callback_get_internet_status(request, response)

        self.assertTrue(self.mock_node.internet_status)

        self.assertEqual(response.output, "True")


if __name__ == "__main__":
    unittest.main()
