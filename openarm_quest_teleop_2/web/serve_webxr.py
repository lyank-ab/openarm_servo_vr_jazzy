#!/usr/bin/env python3
"""
Serve the WebXR teleop page over HTTPS.

WebXR requires a secure context (HTTPS) except on localhost.
This creates a self-signed cert and serves the page.

Usage:
    python3 serve_webxr.py [port]

    Then on Quest 3 browser, navigate to:
        https://<YOUR_XPS_IP>:8443

    You'll need to accept the self-signed certificate warning.
"""

import http.server
import os
import ssl
import subprocess
import sys

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8443
CERT_DIR = os.path.expanduser("~/.openarm_teleop_certs")
CERT_FILE = os.path.join(CERT_DIR, "cert.pem")
KEY_FILE = os.path.join(CERT_DIR, "key.pem")
WEBXR_DIR = os.path.dirname(os.path.abspath(__file__))


def generate_cert():
    """Generate a self-signed certificate if it doesn't exist."""
    if os.path.exists(CERT_FILE) and os.path.exists(KEY_FILE):
        print(f"Using existing cert: {CERT_FILE}")
        return

    os.makedirs(CERT_DIR, exist_ok=True)
    print("Generating self-signed certificate...")
    subprocess.run([
        "openssl", "req", "-x509", "-newkey", "rsa:2048",
        "-keyout", KEY_FILE, "-out", CERT_FILE,
        "-days", "365", "-nodes",
        "-subj", "/CN=openarm-teleop/O=AIR-Lab/C=US",
    ], check=True)
    print(f"Certificate saved to {CERT_DIR}")


def get_local_ip():
    """Get the machine's local IP address."""
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


def main():
    generate_cert()

    os.chdir(WEBXR_DIR)

    handler = http.server.SimpleHTTPRequestHandler
    httpd = http.server.HTTPServer(("0.0.0.0", PORT), handler)

    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(CERT_FILE, KEY_FILE)
    httpd.socket = ctx.wrap_socket(httpd.socket, server_side=True)

    local_ip = get_local_ip()
    print(f"\n{'='*60}")
    print("  OpenArm Quest 3 Teleop WebXR Server")
    print(f"  Serving: {WEBXR_DIR}")
    print(f"  URL:     https://{local_ip}:{PORT}")
    print("")
    print("  On Quest 3 browser, go to:")
    print(f"  https://{local_ip}:{PORT}")
    print("  (Accept the certificate warning)")
    print(f"{'='*60}\n")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down.")
        httpd.shutdown()


if __name__ == "__main__":
    main()
