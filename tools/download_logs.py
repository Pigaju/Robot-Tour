#!/usr/bin/env python3
"""
Robot Tour — SPIFFS Log Downloader & Grapher

Usage:
    python3 tools/download_logs.py                # download, delete, and graph
    python3 tools/download_logs.py --download      # download + delete only
    python3 tools/download_logs.py --graph [FILE]  # graph a previously downloaded CSV
    python3 tools/download_logs.py --watch         # poll for USB connection, auto-download

Requires:  pip install pyserial matplotlib
"""

import argparse
import glob
import os
import re
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial not installed.  Run:  pip install pyserial")

BAUD = 115200
LOGS_DIR = Path(__file__).resolve().parent.parent / "logs"
TIMEOUT_S = 2          # per-line read timeout
DOWNLOAD_TIMEOUT_S = 30  # total timeout for download_all


# ---------------------------------------------------------------------------
# Serial helpers
# ---------------------------------------------------------------------------

def find_device_port():
    """Auto-detect the ESP32 serial port on macOS."""
    for p in serial.tools.list_ports.comports():
        if "usbmodem" in p.device or "usbserial" in p.device or "SLAB" in p.description:
            return p.device
    return None


def open_serial(port=None):
    port = port or find_device_port()
    if not port:
        return None
    try:
        ser = serial.Serial(port, BAUD, timeout=TIMEOUT_S)
        time.sleep(2)  # ESP32-S3 USB CDC needs time after port open
        ser.reset_input_buffer()
        ser.write(b'\n\n')  # flush any partial command in device buffer
        ser.flush()
        time.sleep(0.3)
        ser.reset_input_buffer()
        return ser
    except serial.SerialException as e:
        print(f"Cannot open {port}: {e}")
        return None


def send_cmd(ser, cmd):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    ser.flush()


def ping(ser):
    send_cmd(ser, "ping")
    deadline = time.time() + 3
    while time.time() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line == "PONG":
            return True
    return False


# ---------------------------------------------------------------------------
# Download
# ---------------------------------------------------------------------------

def download_all(ser):
    """Send 'download_all', capture files, return list of saved paths."""
    send_cmd(ser, "download_all")

    saved = []
    current_name = None
    current_lines = []
    deadline = time.time() + DOWNLOAD_TIMEOUT_S
    started = False

    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            if started:
                continue
            break
        line = raw.decode(errors="replace").rstrip("\r\n")

        if line == "DOWNLOAD_ALL_BEGIN":
            started = True
            continue
        if line == "DOWNLOAD_ALL_END":
            break

        m_begin = re.match(r"^FILE_BEGIN\s+(\S+)\s+(\d+)$", line)
        if m_begin:
            current_name = m_begin.group(1).strip("/")
            current_lines = []
            continue

        m_end = re.match(r"^FILE_END\s+(\S+)$", line)
        if m_end and current_name:
            LOGS_DIR.mkdir(parents=True, exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            safe_name = current_name.replace("/", "_")
            dest = LOGS_DIR / f"{ts}_{safe_name}"
            dest.write_text("\n".join(current_lines) + "\n")
            print(f"  Saved {dest.name}  ({len(current_lines)} lines)")
            saved.append(str(dest))
            current_name = None
            current_lines = []
            continue

        if current_name is not None:
            current_lines.append(line)

    if not started:
        print("  No response from device (DOWNLOAD_ALL_BEGIN not received)")
    return saved


def delete_all(ser):
    send_cmd(ser, "delete_all")
    deadline = time.time() + 10
    while time.time() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("DELETE_ALL_END"):
            m = re.search(r"count=(\d+)", line)
            n = int(m.group(1)) if m else "?"
            print(f"  Deleted {n} file(s) from SPIFFS")
            return
        if line.startswith("DELETE "):
            print(f"  {line}")
    print("  delete_all: no confirmation received")


# ---------------------------------------------------------------------------
# Graphing
# ---------------------------------------------------------------------------

def graph_csv(csv_path):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("matplotlib not installed.  Run:  pip install matplotlib")
    import csv as csvmod

    rows = []
    with open(csv_path, newline="") as fh:
        for line in fh:
            if line.startswith("#") or line.strip() == "":
                continue
            # first non-comment line is the header
            reader = csvmod.DictReader([line] + fh.readlines()[:9999],
                                       fieldnames=None)
            # re-parse: header is in `line`
            break
        else:
            print(f"No data in {csv_path}")
            return

    # Re-read properly
    with open(csv_path, newline="") as fh:
        lines = [l for l in fh if not l.startswith("#") and l.strip()]
    if len(lines) < 2:
        print(f"Not enough data rows in {csv_path}")
        return
    reader = csvmod.DictReader(lines)
    for row in reader:
        rows.append(row)

    if not rows:
        print("No sample rows found.")
        return

    cols = list(rows[0].keys())

    def col_floats(name):
        out = []
        for r in rows:
            try:
                out.append(float(r.get(name, "")))
            except (ValueError, TypeError):
                out.append(float("nan"))
        return out

    t = col_floats("t_ms")

    # Define plot groups
    groups = [
        ("Distance", ["meters", "forwardM", "remainingM", "lateralM"]),
        ("Yaw / Heading", ["currentYaw", "targetYaw", "yawErrDeg"]),
        ("Gyro", ["gz_dps", "gzRaw_dps", "gzBias_dps"]),
        ("Speed", ["actualRate", "leftRate", "rightRate", "nomTargetRate", "effTargetRate"]),
        ("PWM", ["basePWM", "lPwm", "rPwm"]),
        ("Speed PID", ["speedErr", "speedI"]),
        ("Steering", ["corrRaw", "corrFiltered", "corrOut", "steerI", "encTrimPwm"]),
        ("Encoder Distance", ["distL", "distR"]),
    ]

    available = set(cols)
    groups = [(title, [c for c in cs if c in available]) for title, cs in groups]
    groups = [(title, cs) for title, cs in groups if cs]

    n = len(groups)
    if n == 0:
        print("No plottable columns found.")
        return

    fig, axes = plt.subplots(n, 1, figsize=(14, 3.2 * n), sharex=True)
    if n == 1:
        axes = [axes]

    for ax, (title, cs) in zip(axes, groups):
        for c in cs:
            ax.plot(t, col_floats(c), label=c, linewidth=0.8)
        ax.set_ylabel(title)
        ax.legend(loc="upper right", fontsize=7, ncol=min(len(cs), 4))
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("t_ms")
    fig.suptitle(Path(csv_path).name, fontsize=10)
    fig.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Watch mode — poll for USB device
# ---------------------------------------------------------------------------

def watch_loop(port=None):
    print("Watching for USB connection... (Ctrl+C to stop)")
    seen = False
    while True:
        p = port or find_device_port()
        if p:
            if not seen:
                seen = True
                print(f"\nDevice detected on {p}")
                ser = open_serial(p)
                if ser and ping(ser):
                    print("Downloading logs...")
                    saved = download_all(ser)
                    if saved:
                        delete_all(ser)
                        print(f"\nGraphing {len(saved)} file(s)...")
                        for f in saved:
                            graph_csv(f)
                    else:
                        print("No log files on device.")
                    ser.close()
                elif ser:
                    print("Ping failed — device not responding.")
                    ser.close()
                else:
                    print("Could not open serial port.")
        else:
            if seen:
                seen = False
                print("Device disconnected. Waiting...")
        time.sleep(2)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Robot Tour log downloader & grapher")
    parser.add_argument("--port", help="Serial port (auto-detected if omitted)")
    parser.add_argument("--download", action="store_true", help="Download + delete logs only")
    parser.add_argument("--graph", nargs="?", const="LATEST", help="Graph a CSV (or latest)")
    parser.add_argument("--watch", action="store_true", help="Poll for USB, auto-download")
    args = parser.parse_args()

    if args.watch:
        watch_loop(args.port)
        return

    if args.graph:
        if args.graph == "LATEST":
            csvs = sorted(glob.glob(str(LOGS_DIR / "*.csv")))
            if not csvs:
                sys.exit(f"No CSV files in {LOGS_DIR}")
            target = csvs[-1]
        else:
            target = args.graph
        print(f"Graphing {target}")
        graph_csv(target)
        return

    # Default: download + delete + graph
    ser = open_serial(args.port)
    if not ser:
        sys.exit("No device found. Plug in the robot or specify --port.")

    if not ping(ser):
        ser.close()
        sys.exit("Device not responding to ping.")

    print("Downloading logs...")
    saved = download_all(ser)
    if saved:
        delete_all(ser)
        ser.close()
        print(f"\nGraphing {len(saved)} file(s)...")
        for f in saved:
            graph_csv(f)
    else:
        print("No log files on device.")
        ser.close()


if __name__ == "__main__":
    main()
