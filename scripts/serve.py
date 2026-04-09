"""Tiny CORS-enabled HTTP server for local model development."""
from http.server import HTTPServer, SimpleHTTPRequestHandler
import os, sys

class CORSHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        super().end_headers()

port = int(sys.argv[1]) if len(sys.argv) > 1 else 8081
os.chdir(os.path.join(os.path.dirname(__file__), "..", "dist"))
print(f"Serving {os.getcwd()} on http://localhost:{port}/ with CORS enabled")
HTTPServer(("", port), CORSHandler).serve_forever()
