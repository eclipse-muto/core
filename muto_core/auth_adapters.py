#
# Copyright (c) 2025 Composiv.ai
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0
#
# Contributors:
#   Composiv.ai - initial API and implementation
#

from __future__ import annotations

import base64
from abc import ABC, abstractmethod

import requests

DEFAULT_AUTH_TYPE = "none"


class AuthAdapter(ABC):
    """Strategy interface for applying authentication to outgoing twin server requests."""

    @abstractmethod
    def apply(self, headers: dict) -> dict | None:
        """Return a copy of `headers` with authentication applied.

        Returns None if authentication could not be established, signalling
        to the caller that the request must not be sent.
        """

    def get_credentials(self) -> dict:
        """Return credential metadata exposed by this adapter, if any.

        Generic contract so callers (e.g. the `get_credentials` service) can
        query whichever adapter is configured without knowing its concrete
        type. Adapters that don't issue explicit credentials (None, Basic)
        return {}.
        """
        return {}


class NoneAuthAdapter(AuthAdapter):
    """No authentication is applied to the request."""

    def apply(self, headers: dict) -> dict | None:
        return dict(headers)


class BasicAuthAdapter(AuthAdapter):
    """Applies HTTP Basic authentication using a username and password."""

    def __init__(self, node):
        node.declare_parameter("user", "")
        node.declare_parameter("password", "")

        self.username = node.get_parameter("user").value
        self.password = node.get_parameter("password").value

    def apply(self, headers: dict) -> dict | None:
        headers = dict(headers)
        credentials = base64.b64encode(f"{self.username}:{self.password}".encode()).decode()
        headers["Authorization"] = f"Basic {credentials}"
        return headers


class OAuth2Adapter(AuthAdapter):
    """Applies OAuth2 authentication by attaching a bearer token to the request.

    The token is obtained from the configured JWT endpoint using the OAuth2
    client credentials grant.
    """

    def __init__(self, node):
        self.node = node

        node.declare_parameter("jwt_url", "")
        node.declare_parameter("jwt_client_id", "")
        node.declare_parameter("jwt_client_secret", "")

        self.jwt_url = node.get_parameter("jwt_url").value
        self.jwt_client_id = node.get_parameter("jwt_client_id").value
        self.jwt_client_secret = node.get_parameter("jwt_client_secret").value

    def get_jwt_token(self) -> dict:
        """
        Fetch a JWT access token using the OAuth2 client credentials flow.

        Sends a POST request to the configured JWT endpoint with the client's
        credentials and returns the parsed JSON response, which typically
        contains the access token along with metadata such as expiry and
        token type.

        Returns:
            dict: The parsed JSON response from the JWT endpoint. Expected
                keys typically include:
                    - access_token (str): The issued JWT.
                    - token_type (str): Usually "Bearer".
                    - expires_in (int): Token lifetime in seconds.

        Raises:
            requests.exceptions.RequestException: If the HTTP request fails
                (e.g., connection error, timeout).
            ValueError: If the response body is not valid JSON.

        Note:
            This method does not call `raise_for_status()`, so HTTP error
            responses (4xx/5xx) will not raise an exception and may instead
            return an error payload as a dict.
        """
        response = requests.post(
            self.jwt_url,
            headers={
                "Content-Type": "application/x-www-form-urlencoded",
            },
            data={
                "grant_type": "client_credentials",
                "client_id": self.jwt_client_id,
                "client_secret": self.jwt_client_secret,
            },
        ).json()

        return response

    def get_credentials(self) -> dict:
        """Fulfil the generic AuthAdapter contract via the OAuth2 client credentials flow."""
        return self.get_jwt_token()

    def apply(self, headers: dict) -> dict | None:
        token = self.get_jwt_token().get("access_token", "")

        if not token:
            self.node.get_logger().error("Error occurred while getting JWT token...")
            return None

        headers = dict(headers)
        headers["Authorization"] = f"Bearer {token}"
        return headers


_ADAPTER_FACTORIES = {
    "none": lambda node: NoneAuthAdapter(),
    "basic": lambda node: BasicAuthAdapter(node),
    "oauth2": lambda node: OAuth2Adapter(
        node,
    ),
}


def create_auth_adapter(node) -> AuthAdapter:
    """Create the AuthAdapter for `auth_type`.

    Falls back to DEFAULT_AUTH_TYPE when `auth_type` is empty or unrecognized.
    New auth methods are added by registering an adapter factory in
    _ADAPTER_FACTORIES.
    """
    factory = _ADAPTER_FACTORIES.get((node.auth_type or "").strip().lower())
    if factory is None:
        factory = _ADAPTER_FACTORIES[DEFAULT_AUTH_TYPE]
    return factory(node)
