#!/usr/bin/env python3
"""
SoleGuard Live Terminal Monitor
Reads real-time telemetry from STM32 RAM over ST-Link SWD and prints it continuously to the terminal.
"""
import subprocess, time, sys, os

OPENOCD = os.path.abspath(".platformio/packages/tool-openocd/bin/openocd")
SCRIPTS = os.path.abspath(".platformio/packages/tool-openocd/openocd/scripts")

def read_telemetry():
    cmd = [
        OPENOCD,
        "-s", SCRIPTS,
        "-f", "interface/stlink.cfg",
        "-c", "adapter speed 1000",
        "-f", "target/stm32f4x.cfg",
        "-c", "init; halt; mdw 0x2000115c 16; resume; shutdown"
    ]
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    return res.stdout

print("=" * 70)
print(" SoleGuard Live Terminal Monitor (SWD Telemetry)")
print(" Reading live sensor stream directly from STM32 Black Pill RAM...")
print("=" * 70)

while True:
    try:
        out = read_telemetry()
        # Parse telemetry
        for line in out.splitlines():
            if line.startswith("0x2000"):
                print(f"[LIVE STREAM] {line}")
        time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nMonitor stopped.")
        break
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(1)
