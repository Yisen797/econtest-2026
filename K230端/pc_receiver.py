"""
K230视频接收器 - 纯内置模块(无需安装opencv/numpy)
收K230 UDP JPEG → 浏览器 http://localhost:8090 看视频
"""
import socket
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

UDP_PORT = 8888
HTTP_PORT = 8090

latest_jpeg = None
lock = threading.Lock()

def udp_receiver():
    global latest_jpeg
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT))
    print(f"UDP listening :{UDP_PORT}")

    while True:
        data, _ = sock.recvfrom(65536)
        if len(data) > 500:
            with lock:
                latest_jpeg = data

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while True:
                with lock:
                    jpeg = latest_jpeg
                if jpeg:
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(f'Content-Length: {len(jpeg)}\r\n\r\n'.encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b'\r\n')
        else:
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            html = '<html><head><meta charset="utf-8"><title>K230 Video</title></head>'
            html += '<body style="background:#000;margin:0;display:flex;justify-content:center;align-items:center;min-height:100vh">'
            html += '<img src="/stream" style="max-width:100%;max-height:100vh"></body></html>'
            self.wfile.write(html.encode())

    def log_message(self, *args): pass  # 静默

def start_http():
    server = HTTPServer(('0.0.0.0', HTTP_PORT), Handler)
    print(f"视频: http://localhost:{HTTP_PORT}")
    print("浏览器打开上面地址, Ctrl+C退出")
    server.serve_forever()

if __name__ == '__main__':
    threading.Thread(target=udp_receiver, daemon=True).start()
    start_http()
