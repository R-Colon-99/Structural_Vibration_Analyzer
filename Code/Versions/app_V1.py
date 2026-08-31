import os
import sys
import time
import signal
import subprocess
import threading
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import serial.tools.list_ports


# ============================================================
# Structural Vibration Analyzer - PC App
# With selectable sensor count
# ============================================================


# -----------------------------
# Project paths
# -----------------------------
APP_DIR = Path(__file__).resolve().parent
CODE_DIR = APP_DIR.parent
PROJECT_ROOT = CODE_DIR.parent

PYTHON_TOOLS_DIR = CODE_DIR / "python_tools"

ANALYZER_SCRIPT = PYTHON_TOOLS_DIR / "analyze_fft.py"
LOGGER_SCRIPT = PYTHON_TOOLS_DIR / "logger_esp32.py"

DATA_DIR = PROJECT_ROOT / "Data"
RAW_DATA_DIR = DATA_DIR / "raw"
PROCESSED_DATA_DIR = DATA_DIR / "processed"

RAW_DATA_DIR.mkdir(parents=True, exist_ok=True)
PROCESSED_DATA_DIR.mkdir(parents=True, exist_ok=True)


# -----------------------------
# Global process handles
# -----------------------------
logger_process = None
analyzer_process = None


# ============================================================
# Helper functions
# ============================================================

def get_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]


def refresh_ports():
    ports = get_serial_ports()
    port_combo["values"] = ports

    if ports:
        selected_port.set(ports[0])
        set_status(f"Found {len(ports)} COM port(s).")
    else:
        selected_port.set("")
        set_status("No COM ports found.")


def set_status(message):
    status_var.set(message)
    root.update_idletasks()


def open_folder(folder_path):
    folder_path = Path(folder_path)

    if not folder_path.exists():
        folder_path.mkdir(parents=True, exist_ok=True)

    os.startfile(folder_path)


def choose_raw_file():
    file_path = filedialog.askopenfilename(
        initialdir=str(RAW_DATA_DIR),
        title="Select Raw Data File",
        filetypes=[
            ("CSV files", "*.csv"),
            ("Text files", "*.txt"),
            ("All files", "*.*")
        ]
    )

    if file_path:
        selected_analysis_file.set(file_path)
        set_status(f"Selected file: {Path(file_path).name}")


def validate_python_tools():
    missing = []

    if not ANALYZER_SCRIPT.exists():
        missing.append(str(ANALYZER_SCRIPT))

    if not LOGGER_SCRIPT.exists():
        missing.append(str(LOGGER_SCRIPT))

    if missing:
        messagebox.showerror(
            "Missing Python Tool",
            "The following required file(s) were not found:\n\n"
            + "\n".join(missing)
        )
        return False

    return True


def selected_sensor_count():
    try:
        return int(num_sensors_var.get())
    except ValueError:
        return 3


# ============================================================
# Logger control
# ============================================================

def start_logger():
    global logger_process

    if logger_process is not None and logger_process.poll() is None:
        messagebox.showwarning("Logger Already Running", "The logger is already running.")
        return

    if not validate_python_tools():
        return

    port = selected_port.get().strip()
    test_name = save_name_var.get().strip()
    baud_rate = baud_var.get().strip()
    num_sensors = selected_sensor_count()

    if not port:
        messagebox.showerror("Missing COM Port", "Select a board / COM port first.")
        return

    if not test_name:
        messagebox.showerror("Missing File Name", "Enter a file name for the saved data.")
        return

    if not test_name.lower().endswith(".csv"):
        test_name += ".csv"

    output_file = RAW_DATA_DIR / test_name

    command = [
        sys.executable,
        str(LOGGER_SCRIPT),
        "--port",
        port,
        "--output",
        str(output_file),
        "--baud",
        baud_rate,
        "--num-sensors",
        str(num_sensors),
    ]

    try:
        logger_process = subprocess.Popen(
            command,
            cwd=str(PYTHON_TOOLS_DIR),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP
        )

        set_status(f"Logger running with {num_sensors} sensor(s). Saving to: {output_file.name}")
        log_to_console(f"\n--- Logger started ---\n")
        log_to_console(f"Sensors selected: {num_sensors}\n")
        log_to_console(f"Command:\n{' '.join(command)}\n\n")

        threading.Thread(
            target=read_process_output,
            args=(logger_process, "LOGGER"),
            daemon=True
        ).start()

    except Exception as e:
        messagebox.showerror("Logger Error", str(e))
        set_status("Logger failed to start.")


def stop_logger():
    global logger_process

    if logger_process is None or logger_process.poll() is not None:
        set_status("Logger is not running.")
        return

    try:
        # Windows-friendly stop
        logger_process.send_signal(signal.CTRL_BREAK_EVENT)
        time.sleep(1)

        if logger_process.poll() is None:
            logger_process.terminate()

        set_status("Logger stopped.")
        log_to_console("\n--- Logger stopped ---\n")

    except Exception as e:
        messagebox.showerror("Stop Logger Error", str(e))


# ============================================================
# Analyzer control
# ============================================================

def run_analyzer():
    global analyzer_process

    if analyzer_process is not None and analyzer_process.poll() is None:
        messagebox.showwarning("Analyzer Already Running", "The analyzer is already running.")
        return

    if not validate_python_tools():
        return

    input_file = selected_analysis_file.get().strip()
    num_sensors = selected_sensor_count()

    if not input_file:
        messagebox.showerror("Missing File", "Select a raw data file to analyze.")
        return

    input_path = Path(input_file)

    if not input_path.exists():
        messagebox.showerror("File Not Found", f"This file does not exist:\n\n{input_path}")
        return

    command = [
        sys.executable,
        str(ANALYZER_SCRIPT),
        "--input",
        str(input_path),
        "--output-dir",
        str(PROCESSED_DATA_DIR),
        "--num-sensors",
        str(num_sensors),
    ]

    try:
        set_status(f"Analyzer running for {num_sensors} sensor(s)...")
        log_to_console(f"\n--- Analyzer started ---\n")
        log_to_console(f"Sensors selected: {num_sensors}\n")
        log_to_console(f"Command:\n{' '.join(command)}\n\n")

        analyzer_process = subprocess.Popen(
            command,
            cwd=str(PYTHON_TOOLS_DIR),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        threading.Thread(
            target=read_process_output,
            args=(analyzer_process, "ANALYZER"),
            daemon=True
        ).start()

        threading.Thread(
            target=wait_for_analyzer,
            daemon=True
        ).start()

    except Exception as e:
        messagebox.showerror("Analyzer Error", str(e))
        set_status("Analyzer failed to start.")


def wait_for_analyzer():
    global analyzer_process

    if analyzer_process is None:
        return

    analyzer_process.wait()

    if analyzer_process.returncode == 0:
        set_status("Analysis complete.")
        log_to_console("\n--- Analyzer complete ---\n")
        messagebox.showinfo(
            "Analysis Complete",
            f"Processed results should be saved in:\n\n{PROCESSED_DATA_DIR}"
        )
    else:
        set_status("Analyzer finished with an error.")
        log_to_console("\n--- Analyzer finished with an error ---\n")


# ============================================================
# Console output
# ============================================================

def log_to_console(text):
    console_box.configure(state="normal")
    console_box.insert(tk.END, text)
    console_box.see(tk.END)
    console_box.configure(state="disabled")


def read_process_output(process, label):
    try:
        for line in process.stdout:
            log_to_console(f"[{label}] {line}")

        for line in process.stderr:
            log_to_console(f"[{label} ERROR] {line}")

    except Exception as e:
        log_to_console(f"[{label} OUTPUT ERROR] {e}\n")


# ============================================================
# UI setup
# ============================================================

root = tk.Tk()
root.title("Structural Vibration Analyzer - PC App")
root.geometry("900x620")
root.minsize(800, 520)

selected_port = tk.StringVar()
save_name_var = tk.StringVar(value="test_001.csv")
baud_var = tk.StringVar(value="921600")
num_sensors_var = tk.StringVar(value="3")
selected_analysis_file = tk.StringVar()
status_var = tk.StringVar(value="Ready.")


# -----------------------------
# Main title
# -----------------------------
title_label = ttk.Label(
    root,
    text="Structural Vibration Analyzer",
    font=("Segoe UI", 18, "bold")
)
title_label.pack(pady=15)


# -----------------------------
# Main frame
# -----------------------------
main_frame = ttk.Frame(root, padding=10)
main_frame.pack(fill="both", expand=True)


# -----------------------------
# Logger section
# -----------------------------
logger_frame = ttk.LabelFrame(main_frame, text="1. Data Logger", padding=10)
logger_frame.pack(fill="x", pady=5)

ttk.Label(logger_frame, text="Board / COM Port:").grid(row=0, column=0, sticky="w", padx=5, pady=5)

port_combo = ttk.Combobox(
    logger_frame,
    textvariable=selected_port,
    state="readonly",
    width=14
)
port_combo.grid(row=0, column=1, sticky="w", padx=5, pady=5)

refresh_button = ttk.Button(
    logger_frame,
    text="Refresh Ports",
    command=refresh_ports
)
refresh_button.grid(row=0, column=2, sticky="w", padx=5, pady=5)

ttk.Label(logger_frame, text="Baud Rate:").grid(row=0, column=3, sticky="w", padx=5, pady=5)

baud_entry = ttk.Entry(
    logger_frame,
    textvariable=baud_var,
    width=12
)
baud_entry.grid(row=0, column=4, sticky="w", padx=5, pady=5)

ttk.Label(logger_frame, text="Sensors:").grid(row=0, column=5, sticky="w", padx=5, pady=5)

num_sensors_combo = ttk.Combobox(
    logger_frame,
    textvariable=num_sensors_var,
    values=["1", "2", "3"],
    state="readonly",
    width=5
)
num_sensors_combo.grid(row=0, column=6, sticky="w", padx=5, pady=5)

ttk.Label(logger_frame, text="Save Raw Data As:").grid(row=1, column=0, sticky="w", padx=5, pady=5)

save_name_entry = ttk.Entry(
    logger_frame,
    textvariable=save_name_var,
    width=35
)
save_name_entry.grid(row=1, column=1, columnspan=2, sticky="w", padx=5, pady=5)

start_logger_button = ttk.Button(
    logger_frame,
    text="Start Logger",
    command=start_logger
)
start_logger_button.grid(row=1, column=3, sticky="w", padx=5, pady=5)

stop_logger_button = ttk.Button(
    logger_frame,
    text="Stop Logger",
    command=stop_logger
)
stop_logger_button.grid(row=1, column=4, sticky="w", padx=5, pady=5)


# -----------------------------
# Analyzer section
# -----------------------------
analyzer_frame = ttk.LabelFrame(main_frame, text="2. Data Analyzer", padding=10)
analyzer_frame.pack(fill="x", pady=10)

ttk.Label(analyzer_frame, text="Raw Data File:").grid(row=0, column=0, sticky="w", padx=5, pady=5)

file_entry = ttk.Entry(
    analyzer_frame,
    textvariable=selected_analysis_file,
    width=70
)
file_entry.grid(row=0, column=1, sticky="we", padx=5, pady=5)

browse_button = ttk.Button(
    analyzer_frame,
    text="Browse",
    command=choose_raw_file
)
browse_button.grid(row=0, column=2, sticky="w", padx=5, pady=5)

run_analyzer_button = ttk.Button(
    analyzer_frame,
    text="Run Analyzer",
    command=run_analyzer
)
run_analyzer_button.grid(row=1, column=1, sticky="w", padx=5, pady=5)

analyzer_frame.columnconfigure(1, weight=1)


# -----------------------------
# Folder shortcuts
# -----------------------------
folder_frame = ttk.LabelFrame(main_frame, text="3. Folders", padding=10)
folder_frame.pack(fill="x", pady=5)

raw_folder_button = ttk.Button(
    folder_frame,
    text="Open Raw Data Folder",
    command=lambda: open_folder(RAW_DATA_DIR)
)
raw_folder_button.grid(row=0, column=0, padx=5, pady=5)

processed_folder_button = ttk.Button(
    folder_frame,
    text="Open Processed Data Folder",
    command=lambda: open_folder(PROCESSED_DATA_DIR)
)
processed_folder_button.grid(row=0, column=1, padx=5, pady=5)

tools_folder_button = ttk.Button(
    folder_frame,
    text="Open Python Tools Folder",
    command=lambda: open_folder(PYTHON_TOOLS_DIR)
)
tools_folder_button.grid(row=0, column=2, padx=5, pady=5)


# -----------------------------
# Console section
# -----------------------------
console_frame = ttk.LabelFrame(main_frame, text="Console Output", padding=10)
console_frame.pack(fill="both", expand=True, pady=10)

console_box = tk.Text(console_frame, height=12, state="disabled", wrap="word")
console_box.pack(fill="both", expand=True)


# -----------------------------
# Status bar
# -----------------------------
status_bar = ttk.Label(
    root,
    textvariable=status_var,
    relief="sunken",
    anchor="w",
    padding=5
)
status_bar.pack(fill="x", side="bottom")


# -----------------------------
# Initial checks
# -----------------------------
refresh_ports()

log_to_console("Structural Vibration Analyzer PC App loaded.\n\n")
log_to_console(f"Project root:\n{PROJECT_ROOT}\n\n")
log_to_console(f"Python tools folder:\n{PYTHON_TOOLS_DIR}\n\n")
log_to_console(f"Analyzer script:\n{ANALYZER_SCRIPT}\n")
log_to_console(f"Logger script:\n{LOGGER_SCRIPT}\n\n")
log_to_console(f"Raw data folder:\n{RAW_DATA_DIR}\n")
log_to_console(f"Processed data folder:\n{PROCESSED_DATA_DIR}\n\n")
log_to_console("Sensor count selection is available in the Data Logger section.\n")
log_to_console("Important: the firmware packet format must match the selected number of sensors.\n\n")

if not ANALYZER_SCRIPT.exists():
    log_to_console("WARNING: analyze_fft.py was not found.\n")

if not LOGGER_SCRIPT.exists():
    log_to_console("WARNING: logger_esp32.py was not found.\n")


# -----------------------------
# Start app
# -----------------------------
root.mainloop()
