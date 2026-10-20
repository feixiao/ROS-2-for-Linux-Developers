#!/usr/bin/env python3
"""
Lightweight Web & Token Server for LiveKit Cockpit Frontend.
Runs on port 8088. Serves index.html and generates LiveKit JWT tokens on demand.
"""

import os
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from token_generator import create_token


class CockpitWebHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == '/' or path == '/index.html':
            html_path = os.path.join(os.path.dirname(__file__), 'web', 'index.html')
            with open(html_path, 'rb') as f:
                content = f.read()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        elif path == '/token':
            query = parse_qs(parsed.query)
            identity = query.get('identity', ['pilot_user'])[0]
            room = query.get('room', ['robot_cockpit'])[0]

            token = create_token(
                api_key="devkey",
                api_secret="secret",
                identity=identity,
                room_name=room,
                can_publish=False
            )
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(token.encode('utf-8'))

        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass


def main():
    port = 8088
    server = ThreadingHTTPServer(('0.0.0.0', port), CockpitWebHandler)
    print("==================================================================")
    print(f">>> LiveKit Cockpit Web Console ready: http://localhost:{port} <<<")
    print("==================================================================")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
