import shutil
from pathlib import Path
from datetime import datetime

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# SETTINGS
# ============================================================

RAW_DATA_DIR = Path(r"F:\RLenovo99\Documents\Projects\Structural_Vibration_Analyzer\Data\raw")
PROCESSED_DIR = Path(r"F:\RLenovo99\Documents\Projects\Structural_Vibration_Analyzer\Data\processed")

FILE_PATTERN = "esp32_vibration_*.csv"

NUM_SENSORS = 3
SENSOR_NAMES = [f"s{i}" for i in range(1, NUM_SENSORS + 1)]
AXIS_SUFFIXES = ["ax_g", "ay_g", "az_g"]
AXIS_LABELS = {
    "ax_g": "X Axis",
    "ay_g": "Y Axis",
    "az_g": "Z Axis",
}

PROCESSED_DIR.mkdir(parents=True, exist_ok=True)

# ============================================================
# SELECT RAW CSV FILE
# ============================================================

csv_files = sorted(RAW_DATA_DIR.glob(FILE_PATTERN))

if not csv_files:
    raise FileNotFoundError(f"No ESP32 vibration CSV files found in: {RAW_DATA_DIR}")

print("\nAvailable raw data files:\n")

for i, file in enumerate(csv_files, start=1):
    print(f"{i}. {file.name}")

choice = input(
    "\nEnter the file number to analyze, or press ENTER to use the latest file: "
).strip()

if choice == "":
    CSV_FILE = max(csv_files, key=lambda p: p.stat().st_mtime)
else:
    try:
        selected_index = int(choice) - 1
        CSV_FILE = csv_files[selected_index]
    except (ValueError, IndexError):
        raise ValueError("Invalid selection. Please enter a valid file number.")

print(f"\nAnalyzing:\n{CSV_FILE}")

# ============================================================
# CREATE UNIQUE PROCESSED RUN FOLDER
# ============================================================

raw_stem = CSV_FILE.stem
run_folder = PROCESSED_DIR / f"{raw_stem}_processed"

if run_folder.exists():
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    run_folder = PROCESSED_DIR / f"{raw_stem}_processed_{timestamp}"

run_folder.mkdir(parents=True, exist_ok=True)

print(f"\nSaving processed results to:\n{run_folder}")

# ============================================================
# LOAD DATA
# ============================================================

df = pd.read_csv(CSV_FILE)

multi_sensor_columns = [
    f"{sensor}_{axis}"
    for sensor in SENSOR_NAMES
    for axis in AXIS_SUFFIXES
]

legacy_columns = ["ax_g", "ay_g", "az_g"]

if all(column in df.columns for column in multi_sensor_columns):
    data_mode = "three_sensor"
    columns_to_analyze = multi_sensor_columns
elif all(column in df.columns for column in legacy_columns):
    data_mode = "single_sensor_legacy"
    columns_to_analyze = legacy_columns
    SENSOR_NAMES = ["s1"]
    NUM_SENSORS = 1
else:
    required_options = multi_sensor_columns + legacy_columns
    missing_columns = [column for column in required_options if column not in df.columns]
    raise ValueError(
        "CSV file does not contain the expected acceleration columns. "
        f"Missing examples: {missing_columns[:10]}"
    )

if "t_us" not in df.columns:
    raise ValueError("Missing required column in CSV file: t_us")

# Copy raw file into processed folder for traceability
raw_copy_path = run_folder / "raw_data_copy.csv"
shutil.copy2(CSV_FILE, raw_copy_path)

# ============================================================
# TIME AND SAMPLING INFORMATION
# ============================================================

t = df["t_us"].to_numpy() / 1_000_000.0
t = t - t[0]

dt_values = np.diff(t)
dt = np.mean(dt_values)
fs = 1 / dt
duration = t[-1] - t[0]
num_samples = len(t)

print(f"\nData mode: {data_mode}")
print(f"Estimated sampling rate: {fs:.2f} Hz")
print(f"Duration: {duration:.3f} s")
print(f"Samples: {num_samples}")

# ============================================================
# ANALYSIS FUNCTIONS
# ============================================================

def centered_signal(column_name):
    signal_raw = df[column_name].to_numpy()
    return signal_raw - np.mean(signal_raw)


def compute_fft(signal_centered):
    window = np.hanning(len(signal_centered))
    signal_windowed = signal_centered * window

    fft_values = np.fft.rfft(signal_windowed)
    freqs = np.fft.rfftfreq(len(signal_windowed), d=dt)
    magnitude = np.abs(fft_values) * 2 / len(signal_windowed)

    return freqs, magnitude


def analyze_column(column_name):
    signal = centered_signal(column_name)

    rms_g = np.sqrt(np.mean(signal ** 2))
    peak_g = np.max(np.abs(signal))
    peak_to_peak_g = np.max(signal) - np.min(signal)

    freqs, magnitude = compute_fft(signal)

    peak_index = np.argmax(magnitude[1:]) + 1
    dominant_freq = freqs[peak_index]
    dominant_mag = magnitude[peak_index]

    fft_df = pd.DataFrame({
        "Frequency_Hz": freqs,
        f"{column_name}_Magnitude_g": magnitude,
    })

    fft_csv_path = run_folder / f"fft_results_{column_name}.csv"
    fft_df.to_csv(fft_csv_path, index=False)

    # --------------------------------------------------------
    # Time-domain plot for this sensor-axis
    # --------------------------------------------------------

    plt.figure(figsize=(12, 5))
    plt.plot(t, signal)
    plt.xlabel("Time [s]")
    plt.ylabel(f"{column_name} [g]")
    plt.title(f"Acceleration vs Time - {column_name}")
    plt.grid(True)

    time_plot_path = run_folder / f"time_signal_{column_name}.png"
    plt.savefig(time_plot_path, dpi=300, bbox_inches="tight")
    plt.show()

    # --------------------------------------------------------
    # FFT plot for this sensor-axis
    # --------------------------------------------------------

    plt.figure(figsize=(12, 5))
    plt.plot(freqs, magnitude)
    plt.xlabel("Frequency [Hz]")
    plt.ylabel("Magnitude [g]")
    plt.title(f"FFT Frequency Spectrum - {column_name}")
    plt.grid(True)
    plt.xlim(0, fs / 2)
    plt.xticks(np.arange(0, (fs / 2) + 10, 10))

    fft_plot_path = run_folder / f"fft_plot_{column_name}.png"
    plt.savefig(fft_plot_path, dpi=300, bbox_inches="tight")
    plt.show()

    return {
        "column": column_name,
        "rms_g": rms_g,
        "peak_g": peak_g,
        "peak_to_peak_g": peak_to_peak_g,
        "dominant_freq_hz": dominant_freq,
        "dominant_magnitude_g": dominant_mag,
        "fft_csv": fft_csv_path.name,
        "time_plot": time_plot_path.name,
        "fft_plot": fft_plot_path.name,
    }

# ============================================================
# RUN ANALYSIS FOR EACH SENSOR-AXIS
# ============================================================

summary_results = []

for column_name in columns_to_analyze:
    print(f"\nAnalyzing: {column_name}")
    result = analyze_column(column_name)
    summary_results.append(result)

    print(f"  Dominant frequency: {result['dominant_freq_hz']:.2f} Hz")
    print(f"  Dominant magnitude: {result['dominant_magnitude_g']:.6f} g")
    print(f"  RMS acceleration: {result['rms_g']:.6f} g")
    print(f"  Peak acceleration: {result['peak_g']:.6f} g")

# ============================================================
# SAVE COMBINED FFT CSV FOR EVERY SENSOR-AXIS
# ============================================================

combined_fft_df = None

for column_name in columns_to_analyze:
    signal = centered_signal(column_name)
    freqs, magnitude = compute_fft(signal)

    if combined_fft_df is None:
        combined_fft_df = pd.DataFrame({"Frequency_Hz": freqs})

    combined_fft_df[f"{column_name}_Magnitude_g"] = magnitude

combined_fft_csv_path = run_folder / "fft_results_all_sensor_axes.csv"
combined_fft_df.to_csv(combined_fft_csv_path, index=False)

# Keep the old filename available when using legacy/sensor-1 style workflows.
legacy_fft_csv_path = run_folder / "fft_results_all_axes.csv"
combined_fft_df.to_csv(legacy_fft_csv_path, index=False)

# ============================================================
# SAVE COMBINED TIME-DOMAIN CSV
# ============================================================

time_domain_data = {"Time_s": t}

for column_name in columns_to_analyze:
    time_domain_data[f"{column_name}_centered"] = centered_signal(column_name)

time_domain_df = pd.DataFrame(time_domain_data)
time_domain_csv_path = run_folder / "time_domain_centered_all_sensor_axes.csv"
time_domain_df.to_csv(time_domain_csv_path, index=False)

# Keep the old filename available.
legacy_time_csv_path = run_folder / "time_domain_centered_all_axes.csv"
time_domain_df.to_csv(legacy_time_csv_path, index=False)

# ============================================================
# SAVE COMBINED TIME-DOMAIN PLOTS
# ============================================================

if data_mode == "three_sensor":
    for sensor in SENSOR_NAMES:
        plt.figure(figsize=(12, 6))

        for axis in AXIS_SUFFIXES:
            column_name = f"{sensor}_{axis}"
            plt.plot(t, centered_signal(column_name), label=axis)

        plt.xlabel("Time [s]")
        plt.ylabel("Acceleration [g]")
        plt.title(f"Acceleration vs Time - All Axes - {sensor}")
        plt.grid(True)
        plt.legend()

        plot_path = run_folder / f"time_signal_all_axes_{sensor}.png"
        plt.savefig(plot_path, dpi=300, bbox_inches="tight")
        plt.show()
else:
    plt.figure(figsize=(12, 6))

    for axis in AXIS_SUFFIXES:
        plt.plot(t, centered_signal(axis), label=axis)

    plt.xlabel("Time [s]")
    plt.ylabel("Acceleration [g]")
    plt.title("Acceleration vs Time - All Axes")
    plt.grid(True)
    plt.legend()

    plot_path = run_folder / "time_signal_all_axes.png"
    plt.savefig(plot_path, dpi=300, bbox_inches="tight")
    plt.show()

# ============================================================
# SAVE COMBINED FFT PLOTS
# ============================================================

if data_mode == "three_sensor":
    # New requested plots:
    # 1. X-axis FFT from sensors 1, 2, and 3 overlapped
    # 2. Y-axis FFT from sensors 1, 2, and 3 overlapped
    # 3. Z-axis FFT from sensors 1, 2, and 3 overlapped
    for axis in AXIS_SUFFIXES:
        plt.figure(figsize=(12, 6))

        for sensor in SENSOR_NAMES:
            column_name = f"{sensor}_{axis}"
            plt.plot(
                combined_fft_df["Frequency_Hz"],
                combined_fft_df[f"{column_name}_Magnitude_g"],
                label=sensor,
            )

        plt.xlabel("Frequency [Hz]")
        plt.ylabel("Magnitude [g]")
        plt.title(f"Overlapped FFT - {AXIS_LABELS[axis]} - Sensors 1, 2, and 3")
        plt.grid(True)
        plt.legend()
        plt.xlim(0, fs / 2)
        plt.xticks(np.arange(0, (fs / 2) + 10, 10))

        axis_letter = axis.split("_")[0][-1]
        plot_path = run_folder / f"fft_overlap_{axis_letter}_axis_all_sensors.png"
        plt.savefig(plot_path, dpi=300, bbox_inches="tight")
        plt.show()

    # Also keep one FFT plot per sensor with X/Y/Z overlaid.
    for sensor in SENSOR_NAMES:
        plt.figure(figsize=(12, 6))

        for axis in AXIS_SUFFIXES:
            column_name = f"{sensor}_{axis}"
            plt.plot(
                combined_fft_df["Frequency_Hz"],
                combined_fft_df[f"{column_name}_Magnitude_g"],
                label=axis,
            )

        plt.xlabel("Frequency [Hz]")
        plt.ylabel("Magnitude [g]")
        plt.title(f"FFT Frequency Spectrum - All Axes - {sensor}")
        plt.grid(True)
        plt.legend()
        plt.xlim(0, fs / 2)
        plt.xticks(np.arange(0, (fs / 2) + 10, 10))

        plot_path = run_folder / f"fft_plot_all_axes_{sensor}.png"
        plt.savefig(plot_path, dpi=300, bbox_inches="tight")
        plt.show()
else:
    plt.figure(figsize=(12, 6))

    for axis in AXIS_SUFFIXES:
        plt.plot(
            combined_fft_df["Frequency_Hz"],
            combined_fft_df[f"{axis}_Magnitude_g"],
            label=axis,
        )

    plt.xlabel("Frequency [Hz]")
    plt.ylabel("Magnitude [g]")
    plt.title("FFT Frequency Spectrum - All Axes")
    plt.grid(True)
    plt.legend()
    plt.xlim(0, fs / 2)
    plt.xticks(np.arange(0, (fs / 2) + 10, 10))

    plot_path = run_folder / "fft_plot_all_axes.png"
    plt.savefig(plot_path, dpi=300, bbox_inches="tight")
    plt.show()

# ============================================================
# SAVE SUMMARY FILE
# ============================================================

summary_txt_path = run_folder / "summary.txt"

with open(summary_txt_path, "w") as f:
    f.write("STRUCTURAL VIBRATION ANALYSIS SUMMARY\n")
    f.write("=====================================\n\n")

    f.write(f"Raw file analyzed: {CSV_FILE.name}\n")
    f.write(f"Raw file path: {CSV_FILE}\n")
    f.write(f"Processed folder: {run_folder}\n")
    f.write(f"Data mode: {data_mode}\n\n")

    f.write(f"Estimated sampling rate: {fs:.2f} Hz\n")
    f.write(f"Sampling interval: {dt:.8f} s\n")
    f.write(f"Duration: {duration:.3f} s\n")
    f.write(f"Number of samples: {num_samples}\n")
    f.write(f"Nyquist frequency: {fs / 2:.2f} Hz\n\n")

    f.write("Sensor-Axis Results:\n")
    f.write("--------------------\n\n")

    for result in summary_results:
        f.write(f"{result['column']}:\n")
        f.write(f"  Dominant frequency: {result['dominant_freq_hz']:.2f} Hz\n")
        f.write(f"  Dominant magnitude: {result['dominant_magnitude_g']:.6f} g\n")
        f.write(f"  RMS acceleration: {result['rms_g']:.6f} g\n")
        f.write(f"  Peak acceleration: {result['peak_g']:.6f} g\n")
        f.write(f"  Peak-to-peak acceleration: {result['peak_to_peak_g']:.6f} g\n")
        f.write(f"  FFT CSV: {result['fft_csv']}\n")
        f.write(f"  Time plot: {result['time_plot']}\n")
        f.write(f"  FFT plot: {result['fft_plot']}\n\n")

    f.write("Saved Files:\n")
    f.write("------------\n")
    f.write("raw_data_copy.csv\n")
    f.write("time_domain_centered_all_sensor_axes.csv\n")
    f.write("time_domain_centered_all_axes.csv\n")
    f.write("fft_results_all_sensor_axes.csv\n")
    f.write("fft_results_all_axes.csv\n")

    if data_mode == "three_sensor":
        f.write("fft_overlap_x_axis_all_sensors.png\n")
        f.write("fft_overlap_y_axis_all_sensors.png\n")
        f.write("fft_overlap_z_axis_all_sensors.png\n")
        f.write("time_signal_all_axes_s1.png\n")
        f.write("time_signal_all_axes_s2.png\n")
        f.write("time_signal_all_axes_s3.png\n")
        f.write("fft_plot_all_axes_s1.png\n")
        f.write("fft_plot_all_axes_s2.png\n")
        f.write("fft_plot_all_axes_s3.png\n")

    for result in summary_results:
        f.write(f"{result['fft_csv']}\n")
        f.write(f"{result['time_plot']}\n")
        f.write(f"{result['fft_plot']}\n")

print("\nAnalysis complete.")
print(f"\nProcessed folder created:\n{run_folder}")
print(f"\nSummary saved to:\n{summary_txt_path}")
