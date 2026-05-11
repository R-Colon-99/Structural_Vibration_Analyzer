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

AXES_TO_ANALYZE = ["ax_g", "ay_g", "az_g"]

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

required_columns = ["t_us"] + AXES_TO_ANALYZE

missing_columns = [col for col in required_columns if col not in df.columns]

if missing_columns:
    raise ValueError(f"Missing required columns in CSV file: {missing_columns}")

# Copy raw file into processed folder for traceability
raw_copy_path = run_folder / "raw_data_copy.csv"
shutil.copy2(CSV_FILE, raw_copy_path)

# ============================================================
# TIME AND SAMPLING INFORMATION
# ============================================================

t = df["t_us"].to_numpy() / 1_000_000.0
t = t - t[0]

dt = np.mean(np.diff(t))
fs = 1 / dt
duration = t[-1] - t[0]
num_samples = len(t)

print(f"\nEstimated sampling rate: {fs:.2f} Hz")
print(f"Duration: {duration:.3f} s")
print(f"Samples: {num_samples}")

# ============================================================
# ANALYSIS FUNCTION
# ============================================================

def analyze_axis(axis_name):
    signal_raw = df[axis_name].to_numpy()
    signal_centered = signal_raw - np.mean(signal_raw)

    rms_g = np.sqrt(np.mean(signal_centered ** 2))
    peak_g = np.max(np.abs(signal_centered))
    peak_to_peak_g = np.max(signal_centered) - np.min(signal_centered)

    window = np.hanning(len(signal_centered))
    signal_windowed = signal_centered * window

    fft_values = np.fft.rfft(signal_windowed)
    freqs = np.fft.rfftfreq(len(signal_windowed), d=dt)

    magnitude = np.abs(fft_values) * 2 / len(signal_windowed)

    peak_index = np.argmax(magnitude[1:]) + 1
    dominant_freq = freqs[peak_index]
    dominant_mag = magnitude[peak_index]

    fft_df = pd.DataFrame({
        "Frequency_Hz": freqs,
        f"{axis_name}_Magnitude_g": magnitude
    })

    fft_csv_path = run_folder / f"fft_results_{axis_name}.csv"
    fft_df.to_csv(fft_csv_path, index=False)

    # --------------------------------------------------------
    # Time-domain plot
    # --------------------------------------------------------

    plt.figure(figsize=(12, 5))
    plt.plot(t, signal_centered)
    plt.xlabel("Time [s]")
    plt.ylabel(f"{axis_name} [g]")
    plt.title(f"Acceleration vs Time - {axis_name}")
    plt.grid(True)

    time_plot_path = run_folder / f"time_signal_{axis_name}.png"
    plt.savefig(time_plot_path, dpi=300, bbox_inches="tight")
    plt.show()

    # --------------------------------------------------------
    # FFT plot
    # --------------------------------------------------------

    plt.figure(figsize=(12, 5))
    plt.plot(freqs, magnitude)
    plt.xlabel("Frequency [Hz]")
    plt.ylabel("Magnitude [g]")
    plt.title(f"FFT Frequency Spectrum - {axis_name}")
    plt.grid(True)
    plt.xlim(0, fs / 2)
    plt.xticks(np.arange(0, (fs / 2) + 10, 10))

    fft_plot_path = run_folder / f"fft_plot_{axis_name}.png"
    plt.savefig(fft_plot_path, dpi=300, bbox_inches="tight")
    plt.show()

    return {
        "axis": axis_name,
        "rms_g": rms_g,
        "peak_g": peak_g,
        "peak_to_peak_g": peak_to_peak_g,
        "dominant_freq_hz": dominant_freq,
        "dominant_magnitude_g": dominant_mag,
        "fft_csv": fft_csv_path.name,
        "time_plot": time_plot_path.name,
        "fft_plot": fft_plot_path.name
    }

# ============================================================
# RUN ANALYSIS FOR EACH AXIS
# ============================================================

summary_results = []

for axis in AXES_TO_ANALYZE:
    print(f"\nAnalyzing axis: {axis}")
    result = analyze_axis(axis)
    summary_results.append(result)

    print(f"  Dominant frequency: {result['dominant_freq_hz']:.2f} Hz")
    print(f"  Dominant magnitude: {result['dominant_magnitude_g']:.6f} g")
    print(f"  RMS acceleration: {result['rms_g']:.6f} g")
    print(f"  Peak acceleration: {result['peak_g']:.6f} g")

# ============================================================
# SAVE COMBINED FFT CSV
# ============================================================

combined_fft_df = None

for axis in AXES_TO_ANALYZE:
    signal = df[axis].to_numpy()
    signal = signal - np.mean(signal)

    window = np.hanning(len(signal))
    signal_windowed = signal * window

    fft_values = np.fft.rfft(signal_windowed)
    freqs = np.fft.rfftfreq(len(signal_windowed), d=dt)
    magnitude = np.abs(fft_values) * 2 / len(signal_windowed)

    if combined_fft_df is None:
        combined_fft_df = pd.DataFrame({"Frequency_Hz": freqs})

    combined_fft_df[f"{axis}_Magnitude_g"] = magnitude

combined_fft_csv_path = run_folder / "fft_results_all_axes.csv"
combined_fft_df.to_csv(combined_fft_csv_path, index=False)

# ============================================================
# SAVE COMBINED TIME-DOMAIN CSV
# ============================================================

time_domain_df = pd.DataFrame({
    "Time_s": t,
    "ax_g_centered": df["ax_g"].to_numpy() - np.mean(df["ax_g"].to_numpy()),
    "ay_g_centered": df["ay_g"].to_numpy() - np.mean(df["ay_g"].to_numpy()),
    "az_g_centered": df["az_g"].to_numpy() - np.mean(df["az_g"].to_numpy())
})

time_domain_csv_path = run_folder / "time_domain_centered_all_axes.csv"
time_domain_df.to_csv(time_domain_csv_path, index=False)

# ============================================================
# SAVE COMBINED PLOTS
# ============================================================

plt.figure(figsize=(12, 6))

for axis in AXES_TO_ANALYZE:
    centered_signal = df[axis].to_numpy() - np.mean(df[axis].to_numpy())
    plt.plot(t, centered_signal, label=axis)

plt.xlabel("Time [s]")
plt.ylabel("Acceleration [g]")
plt.title("Acceleration vs Time - All Axes")
plt.grid(True)
plt.legend()

combined_time_plot_path = run_folder / "time_signal_all_axes.png"
plt.savefig(combined_time_plot_path, dpi=300, bbox_inches="tight")
plt.show()

plt.figure(figsize=(12, 6))

for axis in AXES_TO_ANALYZE:
    plt.plot(
        combined_fft_df["Frequency_Hz"],
        combined_fft_df[f"{axis}_Magnitude_g"],
        label=axis
    )

plt.xlabel("Frequency [Hz]")
plt.ylabel("Magnitude [g]")
plt.title("FFT Frequency Spectrum - All Axes")
plt.grid(True)
plt.legend()
plt.xlim(0, fs / 2)
plt.xticks(np.arange(0, (fs / 2) + 10, 10))

combined_fft_plot_path = run_folder / "fft_plot_all_axes.png"
plt.savefig(combined_fft_plot_path, dpi=300, bbox_inches="tight")
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
    f.write(f"Processed folder: {run_folder}\n\n")

    f.write(f"Estimated sampling rate: {fs:.2f} Hz\n")
    f.write(f"Sampling interval: {dt:.8f} s\n")
    f.write(f"Duration: {duration:.3f} s\n")
    f.write(f"Number of samples: {num_samples}\n")
    f.write(f"Nyquist frequency: {fs / 2:.2f} Hz\n\n")

    f.write("Axis Results:\n")
    f.write("-------------\n\n")

    for result in summary_results:
        f.write(f"{result['axis']}:\n")
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
    f.write("time_domain_centered_all_axes.csv\n")
    f.write("fft_results_all_axes.csv\n")
    f.write("time_signal_all_axes.png\n")
    f.write("fft_plot_all_axes.png\n")

    for result in summary_results:
        f.write(f"{result['fft_csv']}\n")
        f.write(f"{result['time_plot']}\n")
        f.write(f"{result['fft_plot']}\n")

print("\nAnalysis complete.")
print(f"\nProcessed folder created:\n{run_folder}")
print(f"\nSummary saved to:\n{summary_txt_path}")