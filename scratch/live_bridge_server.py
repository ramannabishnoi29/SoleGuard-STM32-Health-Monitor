#!/usr/bin/env python3
"""
SoleGuard Real-Time Live Telemetry Server & Single-Foot DFU Ulcer Alert Engine
Connects to STM32F411 Black Pill via SWD / OpenOCD DMA RAM Mirror & USB Serial
Features:
- Dedicated Single Foot Insole Telemetry (Forefoot PA1 & Heel PA2)
- Real-Time 2D Plantar Heatmap on the single foot
- Clinical DFU Ulcer Risk Assessment & Ischemia Load Warning
- Dual Audio Buzzer (PB1) & Haptic Vibration (PB0) Offloading Alarms
"""

import os
import re
import glob
import time
import json
import socket
import threading
import subprocess
from http.server import HTTPServer, BaseHTTPRequestHandler

try:
    import serial
except ImportError:
    serial = None

OPENOCD_BIN = "/Users/ramanpawar9929/Desktop/MPMC/.platformio/packages/tool-openocd/bin/openocd"
TELNET_PORT = 4444
HTTP_PORT = 5050

# Global Single Foot State
live_data = {
    # Foot Pressure Sensors
    "fsr1": 0,          # Forefoot (PA1)
    "fsr2": 0,          # Heel (PA2)
    "volt1": 0.0,
    "volt2": 0.0,

    # Clinical Biometrics
    "temp": 29.2,       # Plantar Skin Temp (TMP117)
    "temp_f": 84.6,
    "heart_rate": 0,    # Heart Rate (MAX30102)
    "ir": 0,

    # DFU Ulcer Metrics
    "ulcer_risk_score": 12,
    "ulcer_tier": "TIER 1: HEALTHY TISSUE",
    "ulcer_alarm": False,
    "ulcer_recommendation": "Pressure load within normal ambulatory threshold.",

    # Actuators
    "buzzer": 0,        # Active Buzzer (PB1)
    "vib_motor": 0,     # Vibration Motor (PB0)

    # Status
    "frame_count": 0,
    "is_connected": False,
    "connection_type": "Searching...",
    "last_update": 0
}
data_lock = threading.Lock()

def process_telemetry_string(ascii_str, source_name="USB Serial"):
    global live_data
    # Parse format: TMP:29.14C | FF:3 HL:2 BUZ:0 | HR:0BPM (IR:520) | N:624
    t_m = re.search(r"TMP:([\d.]+)C", ascii_str)
    ff_m = re.search(r"FF:(\d+)", ascii_str)
    hl_m = re.search(r"HL:(\d+)", ascii_str)
    buz_m = re.search(r"BUZ:(\d+)", ascii_str)
    hr_m = re.search(r"HR:(\d+)", ascii_str)
    ir_m = re.search(r"IR:(\d+)", ascii_str)
    n_m = re.search(r"N:(\d+)", ascii_str)

    if t_m or ff_m:
        with data_lock:
            temp_val = float(t_m.group(1)) if t_m else live_data["temp"]
            fsr1_val = int(ff_m.group(1)) if ff_m else live_data["fsr1"]
            fsr2_val = int(hl_m.group(1)) if hl_m else live_data["fsr2"]
            buz_val = int(buz_m.group(1)) if buz_m else live_data["buzzer"]
            hr_val = int(hr_m.group(1)) if hr_m else live_data["heart_rate"]
            ir_val = int(ir_m.group(1)) if ir_m else live_data["ir"]
            n_val = int(n_m.group(1)) if n_m else live_data["frame_count"]

            live_data["temp"] = temp_val
            live_data["temp_f"] = round(temp_val * 1.8 + 32, 1)
            live_data["fsr1"] = fsr1_val
            live_data["fsr2"] = fsr2_val
            live_data["volt1"] = round((fsr1_val * 3.3) / 4095.0, 2)
            live_data["volt2"] = round((fsr2_val * 3.3) / 4095.0, 2)

            # Single Foot Ulcer Risk Calculation
            # 1. Thermal inflammation factor
            t_score = 0
            if temp_val > 35.5:
                t_score = 45 # Local inflammatory hyperthermia
            elif temp_val > 33.0:
                t_score = 25
            else:
                t_score = 5

            # 2. Ischemic pressure load factor (0 - 50 points)
            max_p = max(fsr1_val, fsr2_val)
            p_score = min(50, int((max_p / 2000.0) * 45))
            total_risk = min(100, max(8, t_score + p_score))
            live_data["ulcer_risk_score"] = total_risk

            # Alarm evaluation: Dual High (>2000 counts both) OR Extreme local overload
            both_high = (fsr1_val >= 2000 and fsr2_val >= 2000)
            single_high = (fsr1_val >= 2000 or fsr2_val >= 2000)
            is_ulcer_impending = both_high or (max_p >= 2500 and temp_val >= 34.0)

            live_data["ulcer_alarm"] = is_ulcer_impending

            if is_ulcer_impending:
                live_data["ulcer_tier"] = "TIER 3: IMPENDING ULCERATION"
                live_data["ulcer_recommendation"] = "CRITICAL: Plantar ischemia overload on both zones! Patient must offload foot immediately!"
                live_data["buzzer"] = 1
                live_data["vib_motor"] = 1
            elif single_high or total_risk >= 40:
                live_data["ulcer_tier"] = "TIER 2: ELEVATED PLANTAR PRESSURE"
                live_data["ulcer_recommendation"] = "Warning: Focal overload detected. Pulsing haptic vibration to guide weight offloading."
                live_data["buzzer"] = 0
                live_data["vib_motor"] = 1
            else:
                live_data["ulcer_tier"] = "TIER 1: HEALTHY TISSUE"
                live_data["ulcer_recommendation"] = "Plantar pressure and skin temperature within normal safe limits."
                live_data["buzzer"] = buz_val
                live_data["vib_motor"] = 0

            live_data["heart_rate"] = hr_val
            live_data["ir"] = ir_val
            live_data["frame_count"] = n_val
            live_data["is_connected"] = True
            live_data["connection_type"] = source_name
            live_data["last_update"] = time.time()

# Background USB Serial CDC Reader
def usb_serial_reader_thread():
    if not serial:
        return

    while True:
        ports = glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.wchusbserial*")
        if ports:
            port_name = ports[0]
            try:
                ser = serial.Serial(port_name, 115200, timeout=1.0)
                print(f"[USB Serial] Connected on {port_name}!")
                while True:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        process_telemetry_string(line, source_name=f"USB Cable ({os.path.basename(port_name)})")
            except Exception as e:
                time.sleep(1.0)
        time.sleep(0.8)

# Background OpenOCD SWD Reader
def openocd_reader_thread():
    global live_data
    openocd_proc = None

    while True:
        with data_lock:
            conn_type = live_data["connection_type"]
            last_up = live_data["last_update"]
        if "USB Cable" in conn_type and (time.time() - last_up < 2.0):
            time.sleep(1.0)
            continue

        sock_test = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        is_running = (sock_test.connect_ex(("127.0.0.1", TELNET_PORT)) == 0)
        sock_test.close()

        if not is_running:
            cmd = [
                OPENOCD_BIN,
                "-f", "interface/stlink.cfg",
                "-f", "target/stm32f4x.cfg",
                "-c", f"gdb port 3333; telnet_port {TELNET_PORT}"
            ]
            try:
                openocd_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                time.sleep(1.8)
            except:
                time.sleep(2.0)
                continue

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2.0)
            s.connect(("127.0.0.1", TELNET_PORT))
            time.sleep(0.1)
            try:
                s.recv(1024)
            except:
                pass

            print("[OpenOCD] Connected to STM32 SWD Bus!")

            while True:
                with data_lock:
                    if "USB Cable" in live_data["connection_type"] and (time.time() - live_data["last_update"] < 2.0):
                        break

                s.sendall(b"mdb 0x200011ac 64\n")
                raw = ""
                deadline = time.time() + 0.3
                while time.time() < deadline:
                    chunk = s.recv(512).decode("utf-8", errors="ignore")
                    raw += chunk
                    if ">" in chunk or "\n\n" in raw:
                        break

                raw_bytes = []
                for line in raw.splitlines():
                    if ":" in line:
                        parts = line.split(":", 1)[1]
                        for b in parts.strip().split():
                            if len(b) == 2 and all(c in "0123456789abcdefABCDEF" for c in b):
                                raw_bytes.append(int(b, 16))

                ascii_str = "".join(chr(b) for b in raw_bytes if b != 0)
                process_telemetry_string(ascii_str, source_name="ST-Link / SWD Cable")
                time.sleep(0.08)

        except Exception as e:
            with data_lock:
                if "USB Cable" not in live_data["connection_type"]:
                    live_data["is_connected"] = False
                    live_data["connection_type"] = "Waiting for Connection..."
            time.sleep(1.5)

DASHBOARD_FILE = "/Users/ramanpawar9929/.gemini/antigravity/brain/26e19fb3-b51d-4273-aca4-4bfc491703fc/soleguard_dashboard.html"

class TelemetryHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        global live_data
        if self.path == "/api/live":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            with data_lock:
                payload = json.dumps(live_data)
            self.wfile.write(payload.encode("utf-8"))
        elif self.path == "/" or self.path == "/index.html":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            if os.path.exists(DASHBOARD_FILE):
                with open(DASHBOARD_FILE, "rb") as f:
                    self.wfile.write(f.read())
            else:
                self.wfile.write(b"Dashboard file not found.")
        else:
            self.send_response(404)
            self.end_headers()

def main():
    t_serial = threading.Thread(target=usb_serial_reader_thread, daemon=True)
    t_serial.start()

    t_openocd = threading.Thread(target=openocd_reader_thread, daemon=True)
    t_openocd.start()

    server = HTTPServer(("0.0.0.0", HTTP_PORT), TelemetryHandler)
    print(f"[SoleGuard Server] Single-Foot Ulcer Hub listening on http://localhost:{HTTP_PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
