#!/usr/bin/env python3
"""Static server that disables browser caching, so every refresh gets the
latest app.js / index.html / solver.js (no more stale-cache 'looks unchanged')."""
import http.server, socketserver, os, sys

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8801
DIRECTORY = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(os.path.abspath(__file__))


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        super().end_headers()


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


if __name__ == '__main__':
    with Server(('127.0.0.1', PORT), NoCacheHandler) as httpd:
        httpd.serve_forever()
