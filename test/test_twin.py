import json
import socket
import unittest
from unittest.mock import MagicMock, patch

import rclpy

from core.twin import Twin


class TestTwin(unittest.TestCase):

    def setUp(self):
        self.node = Twin()
        self.node.twin_url = "http://sandbox.composiv.ai"
        self.node.anonymous = "true"
        self.node.namespace = "org.eclipse.muto.sandbox"
        self.node.name = "composer-test"
        self.node.type = "test_car"
        self.node.unique_name = ""
        self.node.attributes = '{"brand": "muto", "model": "test_model"}'
        self.node.definition = ""
        self.node.topic = "test_topic"
        self.node.thing_id = "test_thing_id"
        self.node.get_logger = MagicMock()

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    @patch.object(Twin, "stack")
    def test_get_current_properties(self, mock_stack):
        self.node.get_current_properties()
        mock_stack.assert_called_once_with("test_thing_id")

    @patch.object(Twin, "stack")
    def test_get_stack_definition(self, mock_stack):
        test_stack_id = "test_stack_id"
        self.node.get_stack_definition(test_stack_id)
        mock_stack.assert_called_once_with("test_stack_id")

    @patch("requests.get")
    def test_stack(self, mock_req):
        returned_value = self.node.stack(self.node.thing_id)

        self.assertIsNone(returned_value)
        mock_req.assert_called_once_with(
            "http://sandbox.composiv.ai/api/2/things/test_thing_id/features/stack"
        )

    @patch("requests.get")
    def test_stack_status_error(self, mock_req):
        mock_response = MagicMock()
        mock_response.status_code = 404
        mock_req.return_value = mock_response
        returned_value = self.node.stack(self.node.thing_id)

        self.assertEqual(returned_value, {})
        mock_req.assert_called_once_with(
            "http://sandbox.composiv.ai/api/2/things/test_thing_id/features/stack"
        )

    @patch("requests.get")
    def test_stack_status_ok(self, mock_req):
        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.text = '{"properties":"test_properties","properties_second":"test_properties_second" }'
        mock_req.return_value = mock_response
        returned_value = self.node.stack(self.node.thing_id)

        self.assertEqual(returned_value, "test_properties")
        mock_req.assert_called_once_with(
            "http://sandbox.composiv.ai/api/2/things/test_thing_id/features/stack"
        )

    @patch("requests.put")
    def test_set_current_stack(self, mock_put):
        test_stack = MagicMock()
        test_stack.manifest = {"stackId": "test_set_current_stack_stackId"}
        self.node.set_current_stack(test_stack)
        self.node.get_logger().info.assert_called_once_with(
            "Setting current stack to {'stackId': 'test_set_current_stack_stackId'}"
        )
        mock_put.assert_called_once_with(
            "http://sandbox.composiv.ai/api/2/things/test_thing_id/features/stack/properties/current",
            headers={"Content-type": "application/json"},
            json={"stackId": "test_set_current_stack_stackId", "state": "unknown"},
        )

    @patch("requests.post")
    @patch("requests.put")
    def test_set_current_stack_post(self, mock_put, mock_post):
        test_stack = MagicMock()
        test_stack.manifest = {
            "stackId": None,
            "stack": [{"thing_id": "test_set_current_stack_thing"}],
        }
        self.node.set_current_stack(test_stack)
        mock_put.assert_not_called()
        mock_post.assert_called_once_with(
            "http://sandbox.composiv.ai/api/2/things/test_thing_id/features/stack/properties/current",
            headers={"Content-type": "application/json"},
            json={"stackId": "test_set_current_stack_thing", "state": "unknown"},
        )

    def test_set_current_stack_none(self):
        returned_value = self.node.set_current_stack(None)
        self.assertIsNone(returned_value)

    def test_get_context(self):
        returned_value = self.node.get_context()
        self.assertEqual(
            returned_value,
            {
                "namespace": "org.eclipse.muto.sandbox",
                "topic": "test_topic",
                "twin_url": "http://sandbox.composiv.ai",
                "type": "test_car",
                "unique_name": "",
                "thing_id": "test_thing_id",
                "anonymous": "true",
            },
        )

    @patch.object(Twin, "device_register_data")
    @patch("requests.put")
    @patch("requests.patch")
    def test_register_device_status_400(
        self, mock_patch, mock_put, mock_device_register_data
    ):
        mock_response_patch = MagicMock()
        mock_response_put = MagicMock()

        mock_response_patch.status_code = 400
        mock_response_put.status_code = 201

        mock_patch.return_value = mock_response_patch
        mock_put.return_value = mock_response_put

        self.node.register_device()

        mock_patch.assert_called_once()
        mock_put.assert_called_once()
        self.assertTrue(self.node.is_device_registered)
        self.node.get_logger().info.assert_called_once_with(
            "Device registered successfully."
        )
        mock_device_register_data.assert_called()

    @patch.object(Twin, "device_register_data")
    @patch("requests.put")
    @patch("requests.patch")
    def test_register_device_status_404(
        self, mock_patch, mock_put, mock_device_register_data
    ):
        mock_response_patch = MagicMock()
        mock_response_put = MagicMock()

        mock_response_patch.status_code = 404
        mock_response_put.status_code = 201

        mock_patch.return_value = mock_response_patch
        mock_put.return_value = mock_response_put

        self.node.register_device()
        mock_patch.assert_called_once()
        mock_put.assert_called_once()
        self.assertTrue(self.node.is_device_registered)
        self.node.get_logger().info.assert_called_once_with(
            "Device registered successfully."
        )
        mock_device_register_data.assert_called()

    @patch.object(Twin, "device_register_data")
    @patch("requests.put")
    @patch("requests.patch")
    def test_register_device_status_201(
        self, mock_patch, mock_put, mock_device_register_data
    ):
        mock_response_patch = MagicMock()

        mock_response_patch.status_code = 201

        mock_patch.return_value = mock_response_patch
        self.node.register_device()
        mock_put.assert_not_called()
        mock_patch.assert_called_once()
        self.assertTrue(self.node.is_device_registered)
        self.node.get_logger().info.assert_called_once_with(
            "Device registered successfully."
        )
        mock_device_register_data.assert_called()

    @patch.object(Twin, "device_register_data")
    @patch("requests.put")
    @patch("requests.patch")
    def test_register_device_status_204(
        self, mock_patch, mock_put, mock_device_register_data
    ):
        mock_response_patch = MagicMock()

        mock_response_patch.status_code = 204

        mock_patch.return_value = mock_response_patch
        self.node.register_device()
        mock_put.assert_not_called()
        mock_patch.assert_called_once()
        self.assertTrue(self.node.is_device_registered)
        self.node.get_logger().info.assert_called_once_with(
            "Device registered successfully."
        )
        mock_device_register_data.assert_called()

    @patch.object(Twin, "device_register_data")
    @patch("requests.put")
    @patch("requests.patch")
    def test_register_device_status_unknown(
        self, mock_patch, mock_put, mock_device_register_data
    ):
        mock_response_patch = MagicMock()

        mock_response_patch.status_code = 300

        mock_patch.return_value = mock_response_patch
        self.node.register_device()
        mock_patch.assert_called_once()
        mock_put.assert_not_called()
        self.assertFalse(self.node.is_device_registered)
        self.node.get_logger().info.assert_not_called()
        self.node.get_logger().warn.assert_called_once_with(
            "Device registration was unsuccessful. Status Code: 300."
        )

        mock_device_register_data.assert_called()

    @patch("requests.get")
    def test_get_registered_telemetries(self, mock_get):
        mock_response_patch = MagicMock()

        mock_response_patch.status_code = 200
        mock_response_patch.text = (
            '{"properties":"test_properties","payload":"test_payload"}'
        )
        mock_get.return_value = mock_response_patch

        self.node.get_registered_telemetries()

        self.node.get_logger().info.assert_called_once_with(
            "Telemetry properties received successfully."
        )

    @patch("requests.get")
    def test_get_registered_telemetries_status_404(self, mock_get):
        mock_response_patch = MagicMock()

        mock_response_patch.status_code = 404
        mock_response_patch.text = (
            '{"properties":"test_properties","payload":"test_payload"}'
        )

        mock_get.return_value = mock_response_patch
        returned_value = self.node.get_registered_telemetries()
        print(returned_value)

        self.assertEqual(
            returned_value, {"properties": "test_properties", "payload": "test_payload"}
        )
        self.node.get_logger().warn.assert_called_once_with(
            'Getting telemetry properties was unsuccessful - 404 {"properties":"test_properties","payload":"test_payload"}.'
        )

    @patch("requests.put")
    @patch.object(Twin, "get_registered_telemetries")
    def test_register_telemetry_201(self, mock_get_registered_telemetries, mock_put):
        test_telemetry = json.dumps({"definition": "test"})
        mock_response = MagicMock()
        mock_response.status_code = 201
        mock_put.return_value = mock_response
        self.node.register_telemetry(test_telemetry)
        mock_get_registered_telemetries.assert_called_once()
        mock_put.assert_called_once_with(
            "http://sandbox.composiv.ai/api/2/things/test_thing_id/features/telemetry/properties/definition",
            headers={"Content-type": "application/json"},
            json=[{"definition": "test"}],
        )
        self.node.get_logger().info.assert_called_once_with(
            "Telemetry registered successfully."
        )

    @patch("requests.put")
    @patch.object(Twin, "get_registered_telemetries")
    def test_register_telemetry_204(self, mock_get_registered_telemetries, mock_put):
        test_telemetry = json.dumps({"definition": "test"})
        mock_response = MagicMock()
        mock_response.status_code = 204
        mock_put.return_value = mock_response
        self.node.register_telemetry(test_telemetry)
        mock_get_registered_telemetries.assert_called_once()
        mock_put.assert_called_once_with(
            "http://sandbox.composiv.ai/api/2/things/test_thing_id/features/telemetry/properties/definition",
            headers={"Content-type": "application/json"},
            json=[{"definition": "test"}],
        )
        self.node.get_logger().info.assert_called_once_with(
            "Telemetry modified successfully."
        )

    @patch("requests.put")
    @patch.object(Twin, "get_registered_telemetries")
    def test_register_telemetry_404(self, mock_get_registered_telemetries, mock_put):
        test_telemetry = json.dumps({"definition": "test"})
        mock_response = MagicMock()
        mock_response.status_code = 404
        mock_put.return_value = mock_response
        self.node.register_telemetry(test_telemetry)
        mock_get_registered_telemetries.assert_called_once()
        mock_put.assert_called_once_with(
            "http://sandbox.composiv.ai/api/2/things/test_thing_id/features/telemetry/properties/definition",
            headers={"Content-type": "application/json"},
            json=[{"definition": "test"}],
        )
        self.node.get_logger().warn.assert_called_once_with(
            "Telemetry registration was unsuccessful - 404."
        )

    @patch("requests.put")
    @patch.object(Twin, "get_registered_telemetries")
    def test_delete_telemetry_204(self, mock_get_registered_telemetries, mock_put):
        test_telemetry = json.dumps({"definition": "test_delete"})
        mock_response = MagicMock()
        mock_response.status_code = 204
        mock_put.return_value = mock_response
        self.node.delete_telemetry(test_telemetry)
        mock_get_registered_telemetries.assert_called_once()
        mock_put.assert_called_once_with(
            "http://sandbox.composiv.ai/api/2/things/test_thing_id/features/telemetry/properties/definition",
            headers={"Content-type": "application/json"},
            json=[],
        )
        self.node.get_logger().info.assert_called_once_with(
            "Telemetry deleted successfully."
        )

    @patch("requests.put")
    @patch.object(Twin, "get_registered_telemetries")
    def test_delete_telemetry_404(self, mock_get_registered_telemetries, mock_put):
        test_telemetry = json.dumps({"definition": "test_delete"})
        mock_response = MagicMock()
        mock_response.status_code = 404
        mock_put.return_value = mock_response
        self.node.delete_telemetry(test_telemetry)
        mock_get_registered_telemetries.assert_called_once()
        mock_put.assert_called_once_with(
            "http://sandbox.composiv.ai/api/2/things/test_thing_id/features/telemetry/properties/definition",
            headers={"Content-type": "application/json"},
            json=[],
        )
        self.node.get_logger().warn.assert_called_once_with(
            "Telemetry deletion was unsuccessful - 404."
        )

    def test_device_register_data(self):
        returned_value = self.node.device_register_data()
        self.assertEqual(
            returned_value,
            {
                "definition": "",
                "attributes": '{"brand": "muto", "model": "test_model"}',
                "features": {
                    "context": {"properties": {}},
                    "stack": {"properties": {}},
                    "telemetry": {"properties": {}},
                },
            },
        )

    @patch("socket.socket")
    def test_connection_failure(self, mock_socket):
        mock_socket.node = MagicMock()
        mock_socket.return_value = mock_socket.node

        self.node.twin_url = "device@server"

        mock_socket.node.connect.side_effect = socket.error("Connection failed")

        with patch.object(self.node, "get_logger") as mock_logger:
            self.node.connection_status()
            mock_logger().warn.assert_called_once_with(
                "Twin Server ping failed: Connection failed"
            )

        self.assertFalse(self.node.internet_status)

    @patch("socket.socket")
    def test_device_registration(self, mock_socket):
        mock_socket.node = MagicMock()
        mock_socket.return_value = mock_socket.node

        self.node.twin_url = "device@server"

        self.node.is_device_registered = False
        with patch.object(self.node, "register_device") as mock_register_device:
            self.node.connection_status()
            mock_register_device.assert_called_once()

        self.node.is_device_registered = True
        with patch.object(self.node, "register_device") as mock_register_device:
            self.node.connection_status()
            mock_register_device.assert_not_called()


if __name__ == "__main__":
    unittest.main()
