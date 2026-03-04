import time
import serial
import csv
import threading
import os
from collections import deque

import matplotlib.pyplot as plt

# ------------------ USER SETTINGS ------------------
PORT = "COM7"       # <-- CHANGE THIS
BAUD = 115200

PLOT_POINTS = 400   # points shown in the live plot window
CAP_TIMEOUT_S = 10  # max seconds to wait for CAP response
# ----------------------------------------------------

# Shared buffers for live plot
raw_buf  = deque(maxlen=PLOT_POINTS)
true_buf = deque(maxlen=PLOT_POINTS)

# Flags / shared state
running = True
cap_request_pending = False
cap_received_value = None
cap_received_event = threading.Event()

# Grid storage (overwrite on repeat)
data_map = {}  # key: (row,col) -> true_avg

# Session metadata (stored in each row)
nRows = int(input("Rows: "))
nCols = int(input("Cols: "))
run_id = input("Run ID (e.g., run01): ").strip() or "run01"
condition = input("Condition (B=baseline, T=blob): ").strip() or "B"
gain_id = input("Gain ID (e.g., 0/1/2): ").strip() or "0"
remark = input("Remark (any notes for this scan): ").strip()

script_dir = os.path.dirname(os.path.abspath(__file__))
filename = os.path.join(script_dir, f"{run_id}_{condition}.csv")
print("Saving to:", filename)
print("Saving to full path:", os.path.abspath(filename))

# --- ADD THIS (creates file immediately) ---
with open(filename, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["run_id","condition","gain_id","remark","row","col","true_avg_V"])
print("CSV created (header written).")
# -----------------------------------------

# Open serial
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(1.0)
ser.reset_input_buffer()

def write_csv():
    """Rewrite CSV from current data_map (overwrites duplicates)."""
    with open(filename, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["run_id","condition","gain_id","remark","row","col","true_avg_V"])
        # write in row-major order for convenience
        for r in range(nRows):
            for c in range(nCols):
                if (r, c) in data_map:
                    w.writerow([run_id, condition, gain_id, remark, r, c, data_map[(r, c)]])

def reader_thread():
    """Continuously read serial. Update plot buffers, and catch CAP lines."""
    global running, cap_received_value

    while running:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue

        # ignore comment/status lines if any
        if line.startswith("#"):
            continue

        # CAP result from Arduino
        if line.startswith("CAP,"):
            try:
                cap_received_value = float(line.split(",", 1)[1].strip())
                cap_received_event.set()
            except:
                pass
            continue

        # skip header
        if line.lower() == "raw,true":
            continue

        # live stream line: raw,true
        parts = line.split(",")
        if len(parts) == 2:
            try:
                raw = float(parts[0])
                tru = float(parts[1])
                raw_buf.append(raw)
                true_buf.append(tru)
            except:
                pass

def command_thread():
    """Handles user commands and triggers capture requests."""
    global running
    row = 0
    col = 0

    print("\nControls:")
    print("  Enter  -> capture this point and advance")
    print("  r      -> repeat this point (overwrite previous value)")
    print("  q      -> quit\n")

    while running and row < nRows:
        cmd = input(f"[{row},{col}] Enter=CAP | r=repeat | q=quit > ").strip().lower()

        if cmd == "q":
            running = False
            break

        # Send capture command to Arduino
        cap_received_event.clear()
        ser.write(b"d")  # If ever needed: b"d\n"

        # Wait for CAP response
        ok = cap_received_event.wait(timeout=CAP_TIMEOUT_S)
        if not ok:
            print("⚠ Timed out waiting for CAP. Try again.")
            continue

        val = cap_received_value
        print(f"CAP received: {val:.6f} V")

        # Overwrite this point
        data_map[(row, col)] = val
        write_csv()
        print(f"Saved to CSV (overwritten if existed): ({row},{col}) = {val:.6f}")

        # Advance only if NOT repeat
        if cmd != "r":
            col += 1
            if col >= nCols:
                col = 0
                row += 1

    if row >= nRows:
        print("\n✅ Grid complete.")
        running = False

# Start background serial reader
t_read = threading.Thread(target=reader_thread, daemon=True)
t_read.start()

# Start command handler
t_cmd = threading.Thread(target=command_thread, daemon=True)
t_cmd.start()

# Live plot in main thread
plt.ion()
fig, ax = plt.subplots()
ln_raw,  = ax.plot([], [], label="raw")
ln_true, = ax.plot([], [], label="true")
ax.set_title("Arduino Live Stream (raw, true)")
ax.set_xlabel("recent sample index")
ax.set_ylabel("Volts")
ax.legend()

try:
    while running:
        if len(raw_buf) > 5:
            x = list(range(len(raw_buf)))
            ln_raw.set_data(x, list(raw_buf))
            ln_true.set_data(x, list(true_buf))
            ax.relim()
            ax.autoscale_view()
        plt.pause(0.05)
finally:
    running = False
    time.sleep(0.2)
    try:
        ser.close()
    except:
        pass
    print("Closed serial. Bye.")