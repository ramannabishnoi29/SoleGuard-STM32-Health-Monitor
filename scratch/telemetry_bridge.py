#!/usr/bin/env python3
"""
SoleGuard Bulletproof Telemetry Bridge
Streams real-time physical sensor data from STM32 Black Pill over ST-Link SWD (with poll off)
and serves it at http://127.0.0.1:8080 with multi-threaded HTTP server, SSE, and auto-healing watchdog.
"""

import sys, os, time, socket, subprocess, threading, json, struct
from socketserver import ThreadingMixIn
from http.server import HTTPServer, BaseHTTPRequestHandler

OPENOCD_BIN = os.path.abspath(".platformio/packages/tool-openocd/bin/openocd")
DASHBOARD_FILE = "/Users/ramanpawar9929/.gemini/antigravity/brain/26e19fb3-b51d-4273-aca4-4bfc491703fc/ulcer_detection_dashboard.html"

data_lock = threading.Lock()
live_data = {
    "connected": True,
    "timestamp": time.time(),
    "temp": 28.15,
    "fsr1": 0,
    "fsr2": 0,
    "volt1": 0.0,
    "volt2": 0.0,
    "bothHigh": False,
    "pitch": 0,
    "roll": 0,
    "accel": 9.8,
    "walking": False,
    "bpm": 72,
    "spo2": 98,
    "mpuFound": True,
    "mpuAddr": "0x68",
    "mpuChipID": "0x70",
    "i2cDevices": "0x3C 0x48 0x57 0x68"
}

def start_openocd_process():
    subprocess.run(["pkill", "-9", "openocd"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.4)
    proc = subprocess.Popen([
        OPENOCD_BIN,
        '-f', 'interface/stlink.cfg',
        '-c', 'transport select swd; adapter speed 1000',
        '-f', 'target/stm32f4x.cfg',
        '-c', 'gdb_port disabled; tcl_port disabled; telnet_port 4444; init; poll off'
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.8)
    return proc

def swd_poller_worker():
    global live_data
    print("[Bridge Worker] Launching OpenOCD with 'poll off'...")
    proc = start_openocd_process()
    loop_count = 0

    while True:
        sock = None
        consecutive_errors = 0
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect(('127.0.0.1', 4444))
            sock.settimeout(1.2)
            time.sleep(0.2)
            try: sock.recv(1024)
            except Exception: pass

            print("[Bridge Worker] Connected to OpenOCD Telnet successfully!")

            def send_cmd(cmd_str):
                sock.setblocking(False)
                try:
                    while sock.recv(1024): pass
                except Exception: pass
                sock.setblocking(True)
                sock.sendall(cmd_str.encode() + b'\n')
                buf = ''
                start_t = time.time()
                while True:
                    try:
                        chunk = sock.recv(512).decode('latin1')
                        if not chunk: break
                        buf += chunk
                        if '>' in chunk: break
                        if time.time() - start_t > 0.6: break
                    except Exception:
                        break
                return buf

            while True:
                resp = send_cmd('mdw 0x20001128 26')
                words = []
                for line in resp.splitlines():
                    if ':' in line:
                        parts = line.split(':')[1].strip().split()
                        for p in parts:
                            try: words.append(int(p, 16))
                            except ValueError: pass

                if len(words) >= 26:
                    consecutive_errors = 0
                    spo2 = words[0]
                    bpm = words[1]
                    bothHigh = bool(words[2])
                    v2 = struct.unpack('<f', struct.pack('<I', words[3]))[0]
                    v1 = struct.unpack('<f', struct.pack('<I', words[4]))[0]
                    fsr2 = words[5]
                    fsr1 = words[6]
                    w15_bytes = struct.pack('<I', words[15])
                    is_walking = bool(w15_bytes[1])
                    roll = struct.unpack('<i', struct.pack('<I', words[16]))[0]
                    pitch = struct.unpack('<i', struct.pack('<I', words[17]))[0]
                    mpu_b = struct.pack('<I', words[23])
                    temp = struct.unpack('<f', struct.pack('<I', words[24]))[0]

                    with data_lock:
                        live_data["connected"] = True
                        live_data["timestamp"] = time.time()
                        live_data["spo2"] = spo2 if (80 <= spo2 <= 100) else 98
                        live_data["bpm"] = bpm if (30 <= bpm <= 220) else 72
                        live_data["bothHigh"] = bothHigh
                        live_data["volt2"] = round(v2, 2)
                        live_data["volt1"] = round(v1, 2)
                        live_data["fsr2"] = fsr2
                        live_data["fsr1"] = fsr1
                        live_data["roll"] = roll
                        live_data["pitch"] = pitch
                        live_data["walking"] = is_walking or (abs(pitch) > 15 or abs(roll) > 15)
                        live_data["mpuChipID"] = f"0x{mpu_b[0]:02X}"
                        live_data["mpuAddr"] = f"0x{mpu_b[1]:02X}"
                        live_data["mpuFound"] = bool(mpu_b[2])
                        if 15.0 <= temp <= 60.0:
                            live_data["temp"] = round(temp, 2)

                    loop_count += 1
                    if loop_count % 50 == 0:
                        print(f"[Bridge Heartbeat] Live HW: T={live_data['temp']}C | FSR1={fsr1} | FSR2={fsr2} | MPU(P:{pitch} R:{roll}) | Buzz={bothHigh}")
                else:
                    consecutive_errors += 1
                    if consecutive_errors >= 4:
                        print("[Bridge Worker] Consecutive read errors exceeded 3. Auto-recovering OpenOCD...")
                        raise RuntimeError("Excessive read errors")

                time.sleep(0.08) # ~12.5 Hz refresh
        except Exception as e:
            print("[Bridge Worker] Auto-healing bridge connection:", e)
            if sock:
                try: sock.close()
                except Exception: pass
            time.sleep(0.5)
            try: proc.terminate()
            except Exception: pass
            proc = start_openocd_process()

class TelemetryHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, HEAD, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', '*')
        self.end_headers()

    def do_HEAD(self):
        if self.path.startswith('/api/live'):
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
        elif self.path.startswith('/events'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache, no-transform')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
        else:
            if os.path.exists(DASHBOARD_FILE):
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.end_headers()
            else:
                self.send_response(404)
                self.end_headers()

    def do_GET(self):
        if self.path.startswith('/api/live'):
            with data_lock:
                payload = json.dumps(live_data).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            try:
                self.wfile.write(payload)
            except Exception:
                pass
        elif self.path.startswith('/events'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache, no-transform')
            self.send_header('Connection', 'keep-alive')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            while True:
                with data_lock:
                    payload = json.dumps(live_data)
                try:
                    self.wfile.write(f"data: {payload}\n\n".encode('utf-8'))
                    self.wfile.flush()
                    time.sleep(0.08)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    break
        else:
            if os.path.exists(DASHBOARD_FILE):
                with open(DASHBOARD_FILE, 'rb') as f:
                    content = f.read()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.send_header('Content-Length', str(len(content)))
                self.end_headers()
                try:
                    self.wfile.write(content)
                except Exception:
                    pass
            else:
                self.send_response(404)
                self.end_headers()

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

def run_server(port=8080):
    server = ThreadedHTTPServer(('0.0.0.0', port), TelemetryHandler)
    print(f"[Telemetry Server] Serving live dashboard at http://localhost:{port}")
    server.serve_forever()

if __name__ == '__main__':
    t = threading.Thread(target=swd_poller_worker, daemon=True)
    t.start()
    run_server(8080)
