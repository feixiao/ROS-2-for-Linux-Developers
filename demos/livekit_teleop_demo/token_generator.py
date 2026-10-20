#!/usr/bin/env python3
"""
Pure Python LiveKit Token Generator.
Generates HMAC-SHA256 JWT tokens for LiveKit rooms without external dependencies.
"""

import base64
import hashlib
import hmac
import json
import time


def b64url(data):
    if isinstance(data, str):
        data = data.encode('utf-8')
    return base64.urlsafe_b64encode(data).rstrip(b'=').decode('utf-8')


def create_token(api_key="devkey", api_secret="secret", identity="pilot_user", room_name="robot_cockpit", can_publish=True):
    header = {'alg': 'HS256', 'typ': 'JWT'}
    now = int(time.time())
    payload = {
        'iss': api_key,
        'sub': identity,
        'nbf': now - 5,
        'exp': now + 86400,
        'video': {
            'room': room_name,
            'roomJoin': True,
            'canPublish': can_publish,
            'canSubscribe': True,
            'canPublishData': True
        }
    }
    encoded_header = b64url(json.dumps(header))
    encoded_payload = b64url(json.dumps(payload))
    signing_input = f'{encoded_header}.{encoded_payload}'.encode('utf-8')
    signature = hmac.new(api_secret.encode('utf-8'), signing_input, hashlib.sha256).digest()
    encoded_sig = b64url(signature)
    return f'{encoded_header}.{encoded_payload}.{encoded_sig}'


if __name__ == '__main__':
    print("Robot Token:", create_token(identity="robot_camera", room_name="robot_cockpit", can_publish=True))
    print("Pilot Token:", create_token(identity="pilot_operator", room_name="robot_cockpit", can_publish=False))
