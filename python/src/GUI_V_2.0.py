import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import time
import threading
import csv
import os
from datetime import datetime
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np


# ============================================================
# USER SETTINGS
# ============================================================
STEPS_PER_CM = 2000

BAUD_VXM = 9600
BAUD_ARDUINO = 115200

NLEDS = 4
CAP_TIMEOUT_S = 20
PLOT_POINTS = 400

# Direction correction
X_DIR = -1
Y_DIR = -1

# Scan row direction
# If scan Y goes wrong direction, change this only.
Y_SCAN_DIR = -1

MOVE_SETTLE_BASE = 0.5
MOVE_WAIT_SCALE = 1800
scan_matrices = [None for _ in range(NLEDS)]

# ============================================================
# GLOBAL STATE
# ============================================================
vxm_ser = None
arduino_ser = None

x_pos = 0.0
y_pos = 0.0

running_scan = False
arduino_running = False

cap_lock = threading.Lock()
cap_received_values = None
cap_received_event = threading.Event()

voff_buf = [deque(maxlen=PLOT_POINTS) for _ in range(NLEDS)]
von_buf = [deque(maxlen=PLOT_POINTS) for _ in range(NLEDS)]
true_buf = [deque(maxlen=PLOT_POINTS) for _ in range(NLEDS)]

scan_data = []
scan_matrix = None
scan_img = None


# ============================================================
# UTILS
# ============================================================
def now_stamp():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def list_com_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


def refresh_ports():
    ports = list_com_ports()
    vxm_combo["values"] = ports
    arduino_combo["values"] = ports


def safe_float(var, default):
    try:
        return float(var.get())
    except Exception:
        return default


def safe_str(var, default=""):
    s = var.get().strip()
    return s if s else default


# ============================================================
# VXM FUNCTIONS
# ============================================================
def connect_vxm():
    global vxm_ser

    port = vxm_port_var.get().strip()
    if not port:
        messagebox.showerror("VXM error", "Select VXM COM port.")
        return

    try:
        vxm_ser = serial.Serial(port, BAUD_VXM, timeout=1)
        time.sleep(1)
        vxm_status.config(text=f"VXM connected: {port}", fg="green")
        setup_vxm()
    except Exception as e:
        vxm_status.config(text="VXM connection failed", fg="red")
        messagebox.showerror("VXM error", str(e))


def vxm_send(cmd, delay=0.05):
    if vxm_ser is None:
        return
    vxm_ser.write((cmd + "\r").encode())
    time.sleep(delay)


def setup_vxm():
    vxm_send("C")
    vxm_send("F")
    vxm_send("S1M2000")
    vxm_send("S2M2000")
    vxm_send("A1M20")
    vxm_send("A2M20")
    vxm_send("R")
    time.sleep(0.3)


def move_axis(axis, physical_steps):
    global x_pos, y_pos

    if vxm_ser is None:
        messagebox.showerror("VXM error", "Connect VXM first.")
        return

    if axis == 1:
        motor_steps = X_DIR * physical_steps
    elif axis == 2:
        motor_steps = Y_DIR * physical_steps
    else:
        return

    vxm_send("C")
    vxm_send("F")
    vxm_send(f"I{axis}M{int(motor_steps)}")
    vxm_send("R")

    wait_time = MOVE_SETTLE_BASE + abs(physical_steps) / MOVE_WAIT_SCALE
    time.sleep(wait_time)

    if axis == 1:
        x_pos += physical_steps / STEPS_PER_CM
    elif axis == 2:
        y_pos += physical_steps / STEPS_PER_CM

    update_position_label()


def update_position_label():
    pos_label.config(text=f"X: {x_pos:.2f} cm, Y: {y_pos:.2f} cm")


def set_zero():
    global x_pos, y_pos
    x_pos = 0.0
    y_pos = 0.0
    update_position_label()
    scan_status.config(text="Current position set as (0,0)", fg="blue")


def return_to_zero():
    global x_pos, y_pos

    if running_scan:
        messagebox.showwarning("Scan running", "Stop scan before returning to zero.")
        return

    x_back_steps = int(round(-x_pos * STEPS_PER_CM))
    y_back_steps = int(round(-y_pos * STEPS_PER_CM))

    scan_status.config(
        text=f"Returning to zero: X={x_back_steps}, Y={y_back_steps}",
        fg="blue"
    )

    if x_back_steps != 0:
        move_axis(1, x_back_steps)

    if y_back_steps != 0:
        move_axis(2, y_back_steps)

    x_pos = 0.0
    y_pos = 0.0
    update_position_label()
    scan_status.config(text="Returned to (0,0)", fg="green")


def get_jog_steps():
    jog_cm = safe_float(jog_var, 0.5)
    return int(round(jog_cm * STEPS_PER_CM))


def jog_x_plus():
    threading.Thread(target=lambda: move_axis(1, get_jog_steps()), daemon=True).start()


def jog_x_minus():
    threading.Thread(target=lambda: move_axis(1, -get_jog_steps()), daemon=True).start()


def jog_y_plus():
    threading.Thread(target=lambda: move_axis(2, get_jog_steps()), daemon=True).start()


def jog_y_minus():
    threading.Thread(target=lambda: move_axis(2, -get_jog_steps()), daemon=True).start()


# ============================================================
# ARDUINO FUNCTIONS
# ============================================================
def connect_arduino():
    global arduino_ser, arduino_running

    port = arduino_port_var.get().strip()
    if not port:
        messagebox.showerror("Arduino error", "Select Arduino COM port.")
        return

    try:
        arduino_ser = serial.Serial(port, BAUD_ARDUINO, timeout=1)
        time.sleep(1.5)
        arduino_ser.reset_input_buffer()

        arduino_running = True
        threading.Thread(target=arduino_reader_thread, daemon=True).start()

        arduino_status.config(text=f"Arduino connected: {port}", fg="green")
    except Exception as e:
        arduino_status.config(text="Arduino connection failed", fg="red")
        messagebox.showerror("Arduino error", str(e))


def arduino_reader_thread():
    global arduino_running, cap_received_values

    while arduino_running:
        try:
            line = arduino_ser.readline().decode(errors="ignore").strip()
        except Exception:
            continue

        if not line:
            continue

        if line.startswith("#"):
            continue

        if line.lower().startswith("led,idx"):
            continue

        if line.startswith("CAP,"):
            try:
                parts = line.split(",")
                vals = [float(x.strip()) for x in parts[1:1 + NLEDS]]
                if len(vals) == NLEDS:
                    with cap_lock:
                        cap_received_values = vals
                    cap_received_event.set()
            except Exception:
                pass
            continue

        if line.startswith("LED,"):
            parts = line.split(",")
            if len(parts) == 5:
                try:
                    idx = int(parts[1])
                    if 1 <= idx <= NLEDS:
                        i = idx - 1
                        voff = float(parts[2])
                        von = float(parts[3])
                        tru = float(parts[4])

                        voff_buf[i].append(voff)
                        von_buf[i].append(von)
                        true_buf[i].append(tru)
                except Exception:
                    pass


def request_cap():
    global cap_received_values

    if arduino_ser is None:
        return None

    with cap_lock:
        cap_received_values = None

    cap_received_event.clear()

    try:
        arduino_ser.write(b"d")
    except Exception:
        return None

    ok = cap_received_event.wait(timeout=CAP_TIMEOUT_S)
    if not ok:
        return None

    with cap_lock:
        vals = cap_received_values

    if vals is None or len(vals) != NLEDS:
        return None

    return vals


# ============================================================
# CSV SAVING
# ============================================================
def get_output_paths():
    run_id = safe_str(run_id_var, "run01")
    condition = safe_str(condition_var, "condition")
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Create main Data folder
    data_root = os.path.join(script_dir, "Data")
    os.makedirs(data_root, exist_ok=True)

    # Create date-wise folder: MM_DD_YYYY
    date_folder = datetime.now().strftime("%m_%d_%Y")
    date_path = os.path.join(data_root, date_folder)
    os.makedirs(date_path, exist_ok=True)

    # File name from run_id and condition
    filename = os.path.join(date_path, f"{run_id}_{condition}.csv")
    backup_filename = os.path.join(date_path, f"{run_id}_{condition}_backup.csv")

    return filename, backup_filename


def write_csv(path):
    run_id = safe_str(run_id_var, "run01")
    condition = safe_str(condition_var, "condition")
    gain_id = safe_str(gain_id_var, "0")
    remark = safe_str(remark_var, "")

    x_size = safe_float(x_size_var, 4.0)
    y_size = safe_float(y_size_var, 4.0)
    step_cm = safe_float(step_var, 0.5)

    with open(path, "w", newline="") as f:
        w = csv.writer(f)

        # Metadata block: written once only
        w.writerow(["METADATA"])
        w.writerow(["timestamp", now_stamp()])
        w.writerow(["run_id", run_id])
        w.writerow(["condition", condition])
        w.writerow(["gain_id", gain_id])
        w.writerow(["remark", remark])
        w.writerow(["x_size_cm", x_size])
        w.writerow(["y_size_cm", y_size])
        w.writerow(["step_cm", step_cm])
        w.writerow(["steps_per_cm", STEPS_PER_CM])
        w.writerow([])

        # Data block
        w.writerow(["DATA"])
        w.writerow([
            "row", "col",
            "x_cm", "y_cm",
            "true1_V", "true2_V", "true3_V", "true4_V"
        ])

        for rec in scan_data:
            vals = rec["true"]
            w.writerow([
                rec["row"], rec["col"],
                rec["x_cm"], rec["y_cm"],
                vals[0], vals[1], vals[2], vals[3]
            ])


def save_all():
    filename, backup_filename = get_output_paths()
    write_csv(filename)
    write_csv(backup_filename)
    return filename


# ============================================================
# LIVE SCAN MAP
# ============================================================
def init_scan_map(Ny, Nx):
    global scan_matrices, scan_img

    x_size = safe_float(x_size_var, 4.0)
    y_size = safe_float(y_size_var, 4.0)

    scan_matrices = [np.full((Ny, Nx), np.nan) for _ in range(NLEDS)]

    scan_ax.clear()

    led_index = int(map_led_var.get()) - 1

    scan_img = scan_ax.imshow(
        scan_matrices[led_index],
        origin="upper",
        aspect="equal",
        extent=[0, x_size, y_size, 0]
    )

    scan_ax.set_title(f"Live scan map: LED {led_index + 1} true_V")
    scan_ax.set_xlabel("X position (cm)")
    scan_ax.set_ylabel("Y position (cm)")

    scan_canvas.draw_idle()


def update_scan_map(row, col, vals):
    global scan_img

    if scan_matrices[0] is None:
        return

    for i in range(NLEDS):
        scan_matrices[i][row - 1, col - 1] = vals[i]

    selected_led = int(map_led_var.get()) - 1
    matrix = scan_matrices[selected_led]

    scan_img.set_data(matrix)

    valid = matrix[~np.isnan(matrix)]
    if valid.size > 0:
        vmin = np.nanmin(valid)
        vmax = np.nanmax(valid)

        if vmin == vmax:
            scan_img.set_clim(vmin - 0.01, vmax + 0.01)
        else:
            scan_img.set_clim(vmin, vmax)

    scan_ax.set_title(f"Live scan map: LED {selected_led + 1} true_V")
    scan_canvas.draw_idle()

def refresh_selected_led_map(event=None):
    if scan_matrices[0] is None or scan_img is None:
        return

    selected_led = int(map_led_var.get()) - 1
    matrix = scan_matrices[selected_led]

    scan_img.set_data(matrix)

    valid = matrix[~np.isnan(matrix)]
    if valid.size > 0:
        vmin = np.nanmin(valid)
        vmax = np.nanmax(valid)

        if vmin == vmax:
            scan_img.set_clim(vmin - 0.01, vmax + 0.01)
        else:
            scan_img.set_clim(vmin, vmax)

    scan_ax.set_title(f"Live scan map: LED {selected_led + 1} true_V")
    scan_canvas.draw_idle()

# ============================================================
# LIVE ARDUINO PLOT
# ============================================================
def open_live_plot_window():
    def plot_loop():
        plt.ion()
        fig, axs = plt.subplots(2, 2, figsize=(10, 7))
        axs = axs.flatten()

        lines = []
        for i in range(NLEDS):
            ax = axs[i]
            l1, = ax.plot([], [], label="Voff")
            l2, = ax.plot([], [], label="Von")
            l3, = ax.plot([], [], label="true")
            ax.set_title(f"LED {i + 1}")
            ax.set_xlabel("Recent index")
            ax.set_ylabel("Volts")
            ax.legend(loc="upper right")
            lines.append((l1, l2, l3))

        fig.suptitle("4-LED Live Stream")
        fig.tight_layout()

        while arduino_running:
            for i in range(NLEDS):
                n = len(true_buf[i])
                if n > 2:
                    x = list(range(n))
                    lines[i][0].set_data(x, list(voff_buf[i]))
                    lines[i][1].set_data(x, list(von_buf[i]))
                    lines[i][2].set_data(x, list(true_buf[i]))
                    axs[i].relim()
                    axs[i].autoscale_view()

            plt.pause(0.05)

    threading.Thread(target=plot_loop, daemon=True).start()


# ============================================================
# SCAN LOGIC
# ============================================================
def scan_thread():
    global running_scan, scan_data

    if vxm_ser is None:
        messagebox.showerror("Error", "Connect VXM first.")
        return

    if arduino_ser is None:
        messagebox.showerror("Error", "Connect Arduino first.")
        return

    running_scan = True
    scan_data = []

    x_size = safe_float(x_size_var, 4.0)
    y_size = safe_float(y_size_var, 4.0)
    step_cm = safe_float(step_var, 0.5)

    step_steps = int(round(step_cm * STEPS_PER_CM))

    Nx = int(round(x_size / step_cm)) + 1
    Ny = int(round(y_size / step_cm)) + 1

    init_scan_map(Ny, Nx)

    filename = save_all()
    scan_status.config(text=f"Scan started. Saving to {os.path.basename(filename)}", fg="blue")

    direction = 1

    for row in range(Ny):
        if not running_scan:
            break

        for col_index in range(Nx):
            if not running_scan:
                break

            if direction == 1:
                col = col_index
            else:
                col = Nx - 1 - col_index

            logical_row = row + 1
            logical_col = col + 1

            x_cm = col * step_cm
            y_cm = row * step_cm

            scan_status.config(
                text=f"Capturing row {logical_row}/{Ny}, col {logical_col}/{Nx} | "
                     f"x={x_cm:.2f} cm, y={y_cm:.2f} cm"
            )

            # Extra settling before capture
            time.sleep(safe_float(dwell_var, 1.0))

            vals = request_cap()

            if vals is None:
                scan_status.config(
                    text=f"CAP failed at row {logical_row}, col {logical_col}. Stopping.",
                    fg="red"
                )
                running_scan = False
                break

            rec = {
                "row": logical_row,
                "col": logical_col,
                "x_cm": x_cm,
                "y_cm": y_cm,
                "true": vals
            }

            scan_data.append(rec)
            save_all()

            update_scan_map(logical_row, logical_col, vals)

            print(
                f"Saved row={logical_row}, col={logical_col}, "
                f"x={x_cm:.2f}, y={y_cm:.2f}, "
                f"LEDs={', '.join([f'{v:.6f}' for v in vals])}"
            )

            if col_index < Nx - 1:
                move_axis(1, direction * step_steps)

        if row < Ny - 1 and running_scan:
            move_axis(2, Y_SCAN_DIR * step_steps)
            direction *= -1

    running_scan = False
    save_all()

    scan_status.config(text="Scan finished", fg="green")

# reset event to avoid stale CAP trigger
cap_received_event.clear()

print("Scan complete. Ready for next scan.")


def start_scan():
    global running_scan

    if running_scan:
        messagebox.showwarning("Scan running", "Scan already in progress.")
        return

    threading.Thread(target=scan_thread, daemon=True).start()


def stop_scan():
    global running_scan
    running_scan = False
    scan_status.config(text="Stopping scan...", fg="red")


# ============================================================
# CLOSE CLEANLY
# ============================================================
def on_closing():
    global running_scan, arduino_running

    running_scan = False
    arduino_running = False

    time.sleep(0.2)

    try:
        if vxm_ser is not None and vxm_ser.is_open:
            vxm_ser.close()
    except Exception:
        pass

    try:
        if arduino_ser is not None and arduino_ser.is_open:
            arduino_ser.close()
    except Exception:
        pass

    root.destroy()

def open_led_live_window():
    if arduino_ser is None:
        messagebox.showwarning("Arduino", "Connect Arduino first.")
        return

    win = tk.Toplevel(root)
    win.title("Live LED V vs Time")

    fig, axs = plt.subplots(2, 2, figsize=(8, 6))
    axs = axs.flatten()

    lines = []
    for i in range(NLEDS):
        ax = axs[i]
        l_true, = ax.plot([], [], label="true")
        ax.set_title(f"LED {i+1}")
        ax.set_xlabel("Recent sample")
        ax.set_ylabel("true_V")
        ax.legend(loc="upper right")
        lines.append(l_true)

    fig.tight_layout()

    canvas = FigureCanvasTkAgg(fig, master=win)
    canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def update_plot():
        if not win.winfo_exists():
            return

        for i in range(NLEDS):
            y = list(true_buf[i])
            if len(y) > 2:
                x = list(range(len(y)))
                lines[i].set_data(x, y)
                axs[i].relim()
                axs[i].autoscale_view()

        canvas.draw_idle()
        win.after(300, update_plot)   # update every 300 ms

    update_plot()
# ============================================================
# GUI
# ============================================================
root = tk.Tk()
root.title("VXM XY Stage + 4-LED Acquisition GUI")

# ---------------- COM frame ----------------
com_frame = tk.LabelFrame(root, text="Connections")
com_frame.grid(row=0, column=0, columnspan=4, padx=8, pady=8, sticky="ew")

tk.Label(com_frame, text="VXM COM").grid(row=0, column=0, padx=5, pady=5)
vxm_port_var = tk.StringVar(value="COM7")
vxm_combo = ttk.Combobox(com_frame, textvariable=vxm_port_var, width=10)
vxm_combo.grid(row=0, column=1, padx=5, pady=5)

tk.Button(com_frame, text="Connect VXM", command=connect_vxm).grid(row=0, column=2, padx=5, pady=5)
vxm_status = tk.Label(com_frame, text="VXM not connected", fg="red")
vxm_status.grid(row=0, column=3, padx=5, pady=5)

tk.Label(com_frame, text="Arduino COM").grid(row=1, column=0, padx=5, pady=5)
arduino_port_var = tk.StringVar(value="COM8")
arduino_combo = ttk.Combobox(com_frame, textvariable=arduino_port_var, width=10)
arduino_combo.grid(row=1, column=1, padx=5, pady=5)

tk.Button(com_frame, text="Connect Arduino", command=connect_arduino).grid(row=1, column=2, padx=5, pady=5)
arduino_status = tk.Label(com_frame, text="Arduino not connected", fg="red")
arduino_status.grid(row=1, column=3, padx=5, pady=5)

tk.Button(com_frame, text="Refresh COM Ports", command=refresh_ports).grid(row=2, column=1, padx=5, pady=5)
#tk.Button(com_frame, text="Open LED Live Plot", command=open_live_plot_window).grid(row=2, column=2, padx=5, pady=5)

# ---------------- Metadata frame ----------------
meta_frame = tk.LabelFrame(root, text="Metadata")
meta_frame.grid(row=1, column=0, columnspan=4, padx=8, pady=8, sticky="ew")

run_id_var = tk.StringVar(value="run01")
condition_var = tk.StringVar(value="B")
gain_id_var = tk.StringVar(value="0")
remark_var = tk.StringVar(value="")

tk.Label(meta_frame, text="Run ID").grid(row=0, column=0, padx=5, pady=5)
tk.Entry(meta_frame, textvariable=run_id_var, width=14).grid(row=0, column=1, padx=5, pady=5)

tk.Label(meta_frame, text="Condition").grid(row=0, column=2, padx=5, pady=5)
tk.Entry(meta_frame, textvariable=condition_var, width=14).grid(row=0, column=3, padx=5, pady=5)

tk.Label(meta_frame, text="Gain ID").grid(row=1, column=0, padx=5, pady=5)
tk.Entry(meta_frame, textvariable=gain_id_var, width=14).grid(row=1, column=1, padx=5, pady=5)

tk.Label(meta_frame, text="Remark").grid(row=1, column=2, padx=5, pady=5)
tk.Entry(meta_frame, textvariable=remark_var, width=30).grid(row=1, column=3, padx=5, pady=5)

# ---------------- Stage frame ----------------
stage_frame = tk.LabelFrame(root, text="Stage Control")
stage_frame.grid(row=2, column=0, columnspan=4, padx=8, pady=8, sticky="ew")

pos_label = tk.Label(stage_frame, text="X: 0.00 cm, Y: 0.00 cm", font=("Arial", 13))
pos_label.grid(row=0, column=0, columnspan=4, padx=5, pady=5)

tk.Label(stage_frame, text="Jog step (cm)").grid(row=1, column=0, padx=5, pady=5)
jog_var = tk.StringVar(value="0.5")
tk.Entry(stage_frame, textvariable=jog_var, width=10).grid(row=1, column=1, padx=5, pady=5)

tk.Button(stage_frame, text="X-", width=10, command=jog_x_minus).grid(row=2, column=0, padx=5, pady=5)
tk.Button(stage_frame, text="X+", width=10, command=jog_x_plus).grid(row=2, column=1, padx=5, pady=5)
tk.Button(stage_frame, text="Y-", width=10, command=jog_y_minus).grid(row=3, column=0, padx=5, pady=5)
tk.Button(stage_frame, text="Y+", width=10, command=jog_y_plus).grid(row=3, column=1, padx=5, pady=5)

tk.Button(stage_frame, text="Set current as (0,0)", command=set_zero).grid(row=2, column=2, padx=5, pady=5)
tk.Button(stage_frame, text="Return to (0,0)", command=return_to_zero).grid(row=3, column=2, padx=5, pady=5)
tk.Button(
    com_frame,
    text="Open LED Live Plot",
    command=open_led_live_window
).grid(row=2, column=2, padx=5, pady=5)
# ---------------- Scan frame ----------------
scan_frame = tk.LabelFrame(root, text="Scan Settings")
scan_frame.grid(row=3, column=0, columnspan=4, padx=8, pady=8, sticky="ew")

x_size_var = tk.StringVar(value="4")
y_size_var = tk.StringVar(value="4")
step_var = tk.StringVar(value="0.5")
dwell_var = tk.StringVar(value="1.0")

tk.Label(scan_frame, text="X size (cm)").grid(row=0, column=0, padx=5, pady=5)
tk.Entry(scan_frame, textvariable=x_size_var, width=10).grid(row=0, column=1, padx=5, pady=5)

tk.Label(scan_frame, text="Y size (cm)").grid(row=1, column=0, padx=5, pady=5)
tk.Entry(scan_frame, textvariable=y_size_var, width=10).grid(row=1, column=1, padx=5, pady=5)

tk.Label(scan_frame, text="Step (cm)").grid(row=2, column=0, padx=5, pady=5)
tk.Entry(scan_frame, textvariable=step_var, width=10).grid(row=2, column=1, padx=5, pady=5)

tk.Label(scan_frame, text="Dwell before CAP (s)").grid(row=3, column=0, padx=5, pady=5)
tk.Entry(scan_frame, textvariable=dwell_var, width=10).grid(row=3, column=1, padx=5, pady=5)

tk.Button(scan_frame, text="Start Scan", width=15, command=start_scan).grid(row=0, column=2, padx=5, pady=5)
tk.Button(scan_frame, text="Stop Scan", width=15, command=stop_scan).grid(row=1, column=2, padx=5, pady=5)

scan_status = tk.Label(scan_frame, text="No scan running", fg="black")
scan_status.grid(row=4, column=0, columnspan=4, padx=5, pady=5)

# ---------------- Map frame ----------------
map_frame = tk.LabelFrame(root, text="Live Scan Map")
map_frame.grid(row=0, column=4, rowspan=4, padx=8, pady=8, sticky="nsew")

map_led_var = tk.StringVar(value="1")

tk.Label(map_frame, text="Display LED map").pack()

map_led_combo = ttk.Combobox(
    map_frame,
    textvariable=map_led_var,
    values=["1", "2", "3", "4"],
    width=5,
    state="readonly"
)
map_led_combo.pack()

map_led_combo.bind("<<ComboboxSelected>>", refresh_selected_led_map)

scan_fig, scan_ax = plt.subplots(figsize=(5, 4))
scan_matrices = [np.full((1, 1), np.nan) for _ in range(NLEDS)]
scan_img = scan_ax.imshow(scan_matrices[0], origin="upper", aspect="equal")
scan_ax.set_title("Live scan map: LED 1 true_V")
scan_ax.set_xlabel("X position (cm)")
scan_ax.set_ylabel("Y position (cm)")
scan_fig.colorbar(scan_img, ax=scan_ax, label="true_V")

scan_canvas = FigureCanvasTkAgg(scan_fig, master=map_frame)
scan_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
refresh_ports()

root.protocol("WM_DELETE_WINDOW", on_closing)

root.mainloop()