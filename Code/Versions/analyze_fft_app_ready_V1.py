# V4 - App-ready FFT analyzer with selectable sensor count
# Structural Vibration Analyzer

import argparse
import shutil
from pathlib import Path
from datetime import datetime

import pandas as pd
import numpy as np

import matplotlib
matplotlib.use("Agg")  # safer when launched from the PC app
import matplotlib.pyplot as plt


# ============================================================
# DEFAULT SETTINGS
# ============================================================

DEFAULT_RAW_DATA_DIR = Path(r"F:\RLenovo99\Documents\Projects\Structural_Vibration_Analyzer\Data\raw")
DEFAULT_PROCESSED_DIR = Path(r"F:\RLenovo99\Documents\Projects\Structural_Vibration_Analyzer\Data\processed")

DEFAULT_FILE_PATTERN = "esp32_vibration_*.csv"

AXIS_SUFFIXES = ["ax_g", "ay_g", "az_g"]
AXIS_LABELS = {
    "ax_g": "X Axis",
    "ay_g": "Y Axis",
    "az_g": "Z Axis",
}

# Sensor 3 is normally mounted on the extruder and treated as the reference/input.
DEFAULT_REFERENCE_SENSOR = "s3"

TRANSMISSIBILITY_EPSILON = 1e-12
NUM_TOP_PEAKS = 10
PEAK_MIN_SPACING_HZ = 2.0

FREQUENCY_BANDS = [
    (0, 10),
    (10, 25),
    (25, 50),
    (50, 100),
    (100, 200),
    (200, None),
]

# Orientation correction.
# Add or edit signs here if you remount sensors differently.
SENSOR_AXIS_SIGN = {
    "s1": {"ax_g": 1,  "ay_g": 1, "az_g": 1},
    "s2": {"ax_g": -1, "ay_g": 1, "az_g": -1},
    "s3": {"ax_g": 1,  "ay_g": 1, "az_g": 1},
}


# ============================================================
# ARGUMENTS
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="FFT analyzer for Structural Vibration Analyzer CSV files."
    )

    parser.add_argument(
        "--input",
        default=None,
        help="Raw CSV file to analyze. If omitted, the script asks or uses latest file.",
    )

    parser.add_argument(
        "--raw-dir",
        default=str(DEFAULT_RAW_DATA_DIR),
        help="Folder containing raw CSV files.",
    )

    parser.add_argument(
        "--output-dir",
        default=str(DEFAULT_PROCESSED_DIR),
        help="Folder where processed run folders are saved.",
    )

    parser.add_argument(
        "--file-pattern",
        default=DEFAULT_FILE_PATTERN,
        help="File pattern used when selecting a raw file automatically.",
    )

    parser.add_argument(
        "--num-sensors",
        type=int,
        choices=[1, 2, 3],
        default=None,
        help="Analyze only sensors s1..sN. If omitted, sensors are detected from the CSV.",
    )

    parser.add_argument(
        "--reference-sensor",
        default=DEFAULT_REFERENCE_SENSOR,
        help="Reference sensor for transmissibility. Default: s3.",
    )

    parser.add_argument(
        "--show-plots",
        action="store_true",
        help="Show Matplotlib windows. Not recommended when running from the PC app.",
    )

    return parser.parse_args()


args = parse_args()

RAW_DATA_DIR = Path(args.raw_dir)
PROCESSED_DIR = Path(args.output_dir)
FILE_PATTERN = args.file_pattern
REFERENCE_SENSOR = args.reference_sensor

PROCESSED_DIR.mkdir(parents=True, exist_ok=True)


# ============================================================
# SELECT RAW CSV FILE
# ============================================================

def select_csv_file():
    if args.input:
        input_path = Path(args.input)
        if not input_path.exists():
            raise FileNotFoundError(f"Input CSV file not found: {input_path}")
        return input_path

    csv_files = sorted(RAW_DATA_DIR.glob(FILE_PATTERN))

    if not csv_files:
        raise FileNotFoundError(f"No ESP32 vibration CSV files found in: {RAW_DATA_DIR}")

    print("\nAvailable raw data files:\n", flush=True)

    for i, file in enumerate(csv_files, start=1):
        print(f"{i}. {file.name}", flush=True)

    choice = input(
        "\nEnter the file number to analyze, or press ENTER to use the latest file: "
    ).strip()

    if choice == "":
        return max(csv_files, key=lambda p: p.stat().st_mtime)

    try:
        selected_index = int(choice) - 1
        return csv_files[selected_index]
    except (ValueError, IndexError):
        raise ValueError("Invalid selection. Please enter a valid file number.")


CSV_FILE = select_csv_file()

print(f"\nAnalyzing:\n{CSV_FILE}", flush=True)


# ============================================================
# CREATE UNIQUE PROCESSED RUN FOLDER
# ============================================================

raw_stem = CSV_FILE.stem
run_folder = PROCESSED_DIR / f"{raw_stem}_processed_V4"

if run_folder.exists():
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    run_folder = PROCESSED_DIR / f"{raw_stem}_processed_V4_{timestamp}"

run_folder.mkdir(parents=True, exist_ok=True)

print(f"\nSaving processed results to:\n{run_folder}", flush=True)


# ============================================================
# LOAD DATA AND DETECT SENSOR COLUMNS
# ============================================================

df = pd.read_csv(CSV_FILE)

legacy_columns = ["ax_g", "ay_g", "az_g"]


def detect_available_sensors(dataframe):
    available = []

    for sensor_number in range(1, 4):
        sensor = f"s{sensor_number}"
        required_columns = [f"{sensor}_{axis}" for axis in AXIS_SUFFIXES]

        if all(column in dataframe.columns for column in required_columns):
            available.append(sensor)

    return available


available_sensors = detect_available_sensors(df)

if args.num_sensors is not None:
    requested_sensors = [f"s{i}" for i in range(1, args.num_sensors + 1)]
    missing_requested = [
        sensor for sensor in requested_sensors
        if sensor not in available_sensors
    ]

    if missing_requested:
        # If the user requested one sensor and legacy columns exist, fall back to legacy.
        if args.num_sensors == 1 and all(column in df.columns for column in legacy_columns):
            data_mode = "single_sensor_legacy"
            SENSOR_NAMES = ["s1"]
            columns_to_analyze = legacy_columns
        else:
            raise ValueError(
                "The CSV file does not contain the selected number of sensor columns. "
                f"Requested: {requested_sensors}. "
                f"Available: {available_sensors}."
            )
    else:
        data_mode = f"{args.num_sensors}_sensor"
        SENSOR_NAMES = requested_sensors
        columns_to_analyze = [
            f"{sensor}_{axis}"
            for sensor in SENSOR_NAMES
            for axis in AXIS_SUFFIXES
        ]

elif available_sensors:
    data_mode = f"{len(available_sensors)}_sensor"
    SENSOR_NAMES = available_sensors
    columns_to_analyze = [
        f"{sensor}_{axis}"
        for sensor in SENSOR_NAMES
        for axis in AXIS_SUFFIXES
    ]

elif all(column in df.columns for column in legacy_columns):
    data_mode = "single_sensor_legacy"
    SENSOR_NAMES = ["s1"]
    columns_to_analyze = legacy_columns

else:
    raise ValueError(
        "CSV file does not contain expected acceleration columns. "
        "Expected either s1_ax_g/s1_ay_g/s1_az_g style columns or legacy ax_g/ay_g/az_g columns."
    )

NUM_SENSORS = len(SENSOR_NAMES)

if "t_us" not in df.columns:
    raise ValueError("Missing required column in CSV file: t_us")

if "sample_index" not in df.columns:
    print("\nWarning: sample_index column not found. Dropped-sample check will be skipped.", flush=True)

# Copy raw file into processed folder for traceability.
raw_copy_path = run_folder / "raw_data_copy.csv"
shutil.copy2(CSV_FILE, raw_copy_path)


# ============================================================
# TIME AND SAMPLING INFORMATION
# ============================================================

t = df["t_us"].to_numpy() / 1_000_000.0
t = t - t[0]

dt_values = np.diff(t)

if len(dt_values) == 0:
    raise ValueError("Not enough samples to analyze. The CSV needs more than one row.")

dt = np.mean(dt_values)
fs = 1 / dt
duration = t[-1] - t[0]
num_samples = len(t)
nyquist = fs / 2

sampling_stats = {
    "mean_dt_s": float(np.mean(dt_values)),
    "min_dt_s": float(np.min(dt_values)),
    "max_dt_s": float(np.max(dt_values)),
    "std_dt_s": float(np.std(dt_values)),
    "estimated_fs_hz": float(fs),
    "nyquist_hz": float(nyquist),
    "duration_s": float(duration),
    "num_samples": int(num_samples),
}

if "sample_index" in df.columns:
    sample_index = df["sample_index"].to_numpy()
    sample_index_diff = np.diff(sample_index)
    dropped_samples = int(np.sum(sample_index_diff - 1))
    repeated_or_reversed = int(np.sum(sample_index_diff <= 0))
    sampling_stats["dropped_samples_estimated"] = dropped_samples
    sampling_stats["repeated_or_reversed_indices"] = repeated_or_reversed
else:
    sampling_stats["dropped_samples_estimated"] = "N/A"
    sampling_stats["repeated_or_reversed_indices"] = "N/A"

print(f"\nData mode: {data_mode}", flush=True)
print(f"Sensors analyzed: {', '.join(SENSOR_NAMES)}", flush=True)
print(f"Estimated sampling rate: {fs:.2f} Hz", flush=True)
print(f"Nyquist frequency: {nyquist:.2f} Hz", flush=True)
print(f"Duration: {duration:.3f} s", flush=True)
print(f"Samples: {num_samples}", flush=True)
print(f"Mean dt: {sampling_stats['mean_dt_s']:.8f} s", flush=True)
print(f"Min dt: {sampling_stats['min_dt_s']:.8f} s", flush=True)
print(f"Max dt: {sampling_stats['max_dt_s']:.8f} s", flush=True)
print(f"Std dt: {sampling_stats['std_dt_s']:.8f} s", flush=True)

if sampling_stats["dropped_samples_estimated"] != "N/A":
    print(f"Estimated dropped samples: {sampling_stats['dropped_samples_estimated']}", flush=True)


# ============================================================
# PLOT HELPER
# ============================================================

def finish_plot(path):
    plt.savefig(path, dpi=300, bbox_inches="tight")
    if args.show_plots:
        plt.show()
    plt.close()


# ============================================================
# SAMPLING QUALITY PLOT
# ============================================================

plt.figure(figsize=(12, 5))
plt.plot(t[1:], dt_values)
plt.xlabel("Time [s]")
plt.ylabel("Sampling interval dt [s]")
plt.title("Sampling Jitter / Time Step Variation")
plt.grid(True)
sampling_jitter_plot_path = run_folder / "sampling_jitter_dt_plot.png"
finish_plot(sampling_jitter_plot_path)

sampling_stats_df = pd.DataFrame([sampling_stats])
sampling_stats_csv_path = run_folder / "sampling_quality_summary.csv"
sampling_stats_df.to_csv(sampling_stats_csv_path, index=False)


# ============================================================
# ANALYSIS FUNCTIONS
# ============================================================

def centered_signal(column_name):
    signal_raw = df[column_name].to_numpy()

    # Apply sensor orientation correction for s1_ax_g style names.
    if "_" in column_name and column_name.startswith("s"):
        sensor, axis = column_name.split("_", 1)
        sign = SENSOR_AXIS_SIGN.get(sensor, {}).get(axis, 1)
        signal_raw = sign * signal_raw

    signal_centered = signal_raw - np.mean(signal_raw)

    # Small noise floor cleanup.
    signal_centered[np.abs(signal_centered) < 0.002] = 0

    return signal_centered


def compute_fft(signal_centered):
    window = np.hanning(len(signal_centered))
    signal_windowed = signal_centered * window

    fft_values = np.fft.rfft(signal_windowed)
    freqs = np.fft.rfftfreq(len(signal_windowed), d=dt)

    magnitude = np.abs(fft_values) * 2 / len(signal_windowed)

    return freqs, magnitude


def find_top_peaks(freqs, magnitude, num_peaks=NUM_TOP_PEAKS, min_spacing_hz=PEAK_MIN_SPACING_HZ):
    if len(freqs) < 3:
        return []

    candidate_indices = []

    for i in range(1, len(magnitude) - 1):
        if magnitude[i] > magnitude[i - 1] and magnitude[i] > magnitude[i + 1]:
            candidate_indices.append(i)

    candidate_indices = sorted(candidate_indices, key=lambda i: magnitude[i], reverse=True)

    selected_indices = []

    for idx in candidate_indices:
        freq = freqs[idx]

        if freq <= 0:
            continue

        too_close = any(abs(freq - freqs[selected]) < min_spacing_hz for selected in selected_indices)

        if not too_close:
            selected_indices.append(idx)

        if len(selected_indices) >= num_peaks:
            break

    peaks = []

    for rank, idx in enumerate(selected_indices, start=1):
        peaks.append({
            "rank": rank,
            "frequency_hz": float(freqs[idx]),
            "magnitude_g": float(magnitude[idx]),
        })

    return peaks


def summarize_frequency_bands(freqs, magnitude, column_name):
    band_rows = []
    total_energy = np.sum(magnitude ** 2)

    for low, high in FREQUENCY_BANDS:
        if high is None:
            mask = freqs >= low
            band_label = f"{low}+ Hz"
        else:
            mask = (freqs >= low) & (freqs < high)
            band_label = f"{low}-{high} Hz"

        band_energy = np.sum(magnitude[mask] ** 2)
        band_rms_like = np.sqrt(np.mean(magnitude[mask] ** 2)) if np.any(mask) else 0
        energy_percent = (band_energy / total_energy * 100) if total_energy > 0 else 0

        band_rows.append({
            "column": column_name,
            "band": band_label,
            "band_energy_g2": float(band_energy),
            "band_rms_like_g": float(band_rms_like),
            "energy_percent": float(energy_percent),
        })

    return band_rows


def analyze_column(column_name):
    signal = centered_signal(column_name)

    rms_g = np.sqrt(np.mean(signal ** 2))
    peak_g = np.max(np.abs(signal))
    peak_to_peak_g = np.max(signal) - np.min(signal)

    freqs, magnitude = compute_fft(signal)

    peak_index = np.argmax(magnitude[1:]) + 1
    dominant_freq = freqs[peak_index]
    dominant_mag = magnitude[peak_index]

    top_peaks = find_top_peaks(freqs, magnitude)
    band_summary = summarize_frequency_bands(freqs, magnitude, column_name)

    fft_df = pd.DataFrame({
        "Frequency_Hz": freqs,
        f"{column_name}_Magnitude_g": magnitude,
    })

    fft_csv_path = run_folder / f"fft_results_{column_name}.csv"
    fft_df.to_csv(fft_csv_path, index=False)

    plt.figure(figsize=(12, 5))
    plt.plot(t, signal)
    plt.xlabel("Time [s]")
    plt.ylabel(f"{column_name} [g]")
    plt.title(f"Acceleration vs Time - {column_name}")
    plt.grid(True)
    time_plot_path = run_folder / f"time_signal_{column_name}.png"
    finish_plot(time_plot_path)

    plt.figure(figsize=(12, 5))
    plt.plot(freqs, magnitude)
    plt.xlabel("Frequency [Hz]")
    plt.ylabel("Magnitude [g]")
    plt.title(f"FFT Frequency Spectrum - {column_name}")
    plt.grid(True)
    plt.xlim(0, nyquist)
    plt.xticks(np.arange(0, nyquist + 25, 25))
    fft_plot_path = run_folder / f"fft_plot_{column_name}.png"
    finish_plot(fft_plot_path)

    return {
        "column": column_name,
        "rms_g": float(rms_g),
        "peak_g": float(peak_g),
        "peak_to_peak_g": float(peak_to_peak_g),
        "dominant_freq_hz": float(dominant_freq),
        "dominant_magnitude_g": float(dominant_mag),
        "fft_csv": fft_csv_path.name,
        "time_plot": time_plot_path.name,
        "fft_plot": fft_plot_path.name,
        "top_peaks": top_peaks,
        "band_summary": band_summary,
    }


# ============================================================
# RUN ANALYSIS FOR EACH SENSOR-AXIS
# ============================================================

summary_results = []
all_peak_rows = []
all_band_rows = []

for column_name in columns_to_analyze:
    print(f"\nAnalyzing: {column_name}", flush=True)
    result = analyze_column(column_name)
    summary_results.append(result)

    print(f"  Dominant frequency: {result['dominant_freq_hz']:.2f} Hz", flush=True)
    print(f"  Dominant magnitude: {result['dominant_magnitude_g']:.6f} g", flush=True)
    print(f"  RMS acceleration: {result['rms_g']:.6f} g", flush=True)
    print(f"  Peak acceleration: {result['peak_g']:.6f} g", flush=True)

    for peak in result["top_peaks"]:
        all_peak_rows.append({
            "column": column_name,
            "rank": peak["rank"],
            "frequency_hz": peak["frequency_hz"],
            "magnitude_g": peak["magnitude_g"],
        })

    all_band_rows.extend(result["band_summary"])

peak_table_csv_path = run_folder / "top_fft_peaks_all_sensor_axes.csv"
pd.DataFrame(all_peak_rows).to_csv(peak_table_csv_path, index=False)

band_summary_csv_path = run_folder / "frequency_band_summary_all_sensor_axes.csv"
pd.DataFrame(all_band_rows).to_csv(band_summary_csv_path, index=False)


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

legacy_time_csv_path = run_folder / "time_domain_centered_all_axes.csv"
time_domain_df.to_csv(legacy_time_csv_path, index=False)


# ============================================================
# TRANSMISSIBILITY ANALYSIS
# ============================================================

transmissibility_rows = []
transmissibility_df = None
transmissibility_csv_path = None
transmissibility_summary_csv_path = None

RESPONSE_SENSORS = [sensor for sensor in SENSOR_NAMES if sensor != REFERENCE_SENSOR]

if (
    len(SENSOR_NAMES) >= 2
    and REFERENCE_SENSOR in SENSOR_NAMES
    and data_mode != "single_sensor_legacy"
):
    print(f"\nCalculating acceleration transmissibility using {REFERENCE_SENSOR} as reference sensor...", flush=True)

    transmissibility_df = pd.DataFrame({
        "Frequency_Hz": combined_fft_df["Frequency_Hz"]
    })

    for axis in AXIS_SUFFIXES:
        reference_column = f"{REFERENCE_SENSOR}_{axis}_Magnitude_g"

        if reference_column not in combined_fft_df.columns:
            continue

        reference_mag = combined_fft_df[reference_column].to_numpy()

        for response_sensor in RESPONSE_SENSORS:
            response_column = f"{response_sensor}_{axis}_Magnitude_g"

            if response_column not in combined_fft_df.columns:
                continue

            response_mag = combined_fft_df[response_column].to_numpy()
            transmissibility = response_mag / np.maximum(reference_mag, TRANSMISSIBILITY_EPSILON)

            trans_column_name = f"T_{response_sensor}_over_{REFERENCE_SENSOR}_{axis}"
            transmissibility_df[trans_column_name] = transmissibility

            valid_mask = combined_fft_df["Frequency_Hz"].to_numpy() > 0
            trans_no_dc = transmissibility[valid_mask]
            freq_no_dc = combined_fft_df["Frequency_Hz"].to_numpy()[valid_mask]

            max_index = np.argmax(trans_no_dc)
            max_transmissibility = trans_no_dc[max_index]
            max_trans_freq = freq_no_dc[max_index]

            transmissibility_rows.append({
                "response_sensor": response_sensor,
                "reference_sensor": REFERENCE_SENSOR,
                "axis": axis,
                "max_transmissibility": float(max_transmissibility),
                "frequency_at_max_transmissibility_hz": float(max_trans_freq),
                "mean_transmissibility": float(np.mean(trans_no_dc)),
                "median_transmissibility": float(np.median(trans_no_dc)),
            })

        plt.figure(figsize=(12, 6))

        for response_sensor in RESPONSE_SENSORS:
            trans_column_name = f"T_{response_sensor}_over_{REFERENCE_SENSOR}_{axis}"

            if trans_column_name in transmissibility_df.columns:
                plt.plot(
                    transmissibility_df["Frequency_Hz"],
                    transmissibility_df[trans_column_name],
                    label=f"{response_sensor}/{REFERENCE_SENSOR}",
                )

        plt.xlabel("Frequency [Hz]")
        plt.ylabel("Acceleration Transmissibility Ratio [-]")
        plt.title(f"Acceleration Transmissibility - {AXIS_LABELS[axis]} - Reference: {REFERENCE_SENSOR}")
        plt.grid(True)
        plt.legend()
        plt.xlim(0, nyquist)
        plt.xticks(np.arange(0, nyquist + 25, 25))

        axis_letter = axis.split("_")[0][-1]
        trans_plot_path = run_folder / f"transmissibility_{axis_letter}_axis_reference_{REFERENCE_SENSOR}.png"
        finish_plot(trans_plot_path)

    transmissibility_csv_path = run_folder / "transmissibility_results_all_axes.csv"
    transmissibility_df.to_csv(transmissibility_csv_path, index=False)

    transmissibility_summary_csv_path = run_folder / "transmissibility_summary.csv"
    pd.DataFrame(transmissibility_rows).to_csv(transmissibility_summary_csv_path, index=False)

else:
    print(
        "\nTransmissibility analysis skipped. "
        "It requires at least 2 selected sensors and the selected reference sensor to exist.",
        flush=True,
    )


# ============================================================
# SAVE COMBINED TIME-DOMAIN PLOTS
# ============================================================

if data_mode != "single_sensor_legacy":
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
        finish_plot(plot_path)

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
    finish_plot(plot_path)


# ============================================================
# SAVE COMBINED FFT PLOTS
# ============================================================

if data_mode != "single_sensor_legacy":
    if len(SENSOR_NAMES) > 1:
        # X, Y, Z FFT from selected sensors overlapped.
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
            plt.title(f"Overlapped FFT - {AXIS_LABELS[axis]} - Selected Sensors")
            plt.grid(True)
            plt.legend()
            plt.xlim(0, nyquist)
            plt.xticks(np.arange(0, nyquist + 25, 25))

            axis_letter = axis.split("_")[0][-1]
            plot_path = run_folder / f"fft_overlap_{axis_letter}_axis_selected_sensors.png"
            finish_plot(plot_path)

    # One FFT plot per sensor with X/Y/Z overlaid.
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
        plt.xlim(0, nyquist)
        plt.xticks(np.arange(0, nyquist + 25, 25))

        plot_path = run_folder / f"fft_plot_all_axes_{sensor}.png"
        finish_plot(plot_path)

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
    plt.xlim(0, nyquist)
    plt.xticks(np.arange(0, nyquist + 25, 25))

    plot_path = run_folder / "fft_plot_all_axes.png"
    finish_plot(plot_path)


# ============================================================
# SAVE SUMMARY FILE
# ============================================================

summary_txt_path = run_folder / "summary_V4.txt"

with open(summary_txt_path, "w") as f:
    f.write("STRUCTURAL VIBRATION ANALYSIS SUMMARY - V4\n")
    f.write("=========================================\n\n")

    f.write(f"Raw file analyzed: {CSV_FILE.name}\n")
    f.write(f"Raw file path: {CSV_FILE}\n")
    f.write(f"Processed folder: {run_folder}\n")
    f.write(f"Data mode: {data_mode}\n")
    f.write(f"Sensors analyzed: {', '.join(SENSOR_NAMES)}\n\n")

    f.write("Sampling Quality:\n")
    f.write("-----------------\n")
    f.write(f"Estimated sampling rate: {fs:.2f} Hz\n")
    f.write(f"Mean sampling interval: {sampling_stats['mean_dt_s']:.8f} s\n")
    f.write(f"Minimum sampling interval: {sampling_stats['min_dt_s']:.8f} s\n")
    f.write(f"Maximum sampling interval: {sampling_stats['max_dt_s']:.8f} s\n")
    f.write(f"Sampling interval standard deviation: {sampling_stats['std_dt_s']:.8f} s\n")
    f.write(f"Duration: {duration:.3f} s\n")
    f.write(f"Number of samples: {num_samples}\n")
    f.write(f"Nyquist frequency: {nyquist:.2f} Hz\n")
    f.write(f"Estimated dropped samples: {sampling_stats['dropped_samples_estimated']}\n")
    f.write(f"Repeated or reversed sample indices: {sampling_stats['repeated_or_reversed_indices']}\n\n")

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
        f.write(f"  FFT plot: {result['fft_plot']}\n")
        f.write("  Top FFT peaks:\n")

        for peak in result["top_peaks"]:
            f.write(
                f"    {peak['rank']}. "
                f"{peak['frequency_hz']:.2f} Hz, "
                f"{peak['magnitude_g']:.6f} g\n"
            )

        f.write("\n")

    if transmissibility_rows:
        f.write("Acceleration Transmissibility Summary:\n")
        f.write("--------------------------------------\n")
        f.write(f"Reference/input sensor: {REFERENCE_SENSOR}\n")
        f.write(f"Response sensors: {', '.join(RESPONSE_SENSORS)}\n\n")

        for row in transmissibility_rows:
            f.write(
                f"{row['response_sensor']}/{row['reference_sensor']} - {row['axis']}: "
                f"Max T = {row['max_transmissibility']:.4f} "
                f"at {row['frequency_at_max_transmissibility_hz']:.2f} Hz, "
                f"Mean T = {row['mean_transmissibility']:.4f}, "
                f"Median T = {row['median_transmissibility']:.4f}\n"
            )

        f.write("\n")

    f.write("Saved Files:\n")
    f.write("------------\n")
    f.write("raw_data_copy.csv\n")
    f.write("sampling_quality_summary.csv\n")
    f.write("sampling_jitter_dt_plot.png\n")
    f.write("time_domain_centered_all_sensor_axes.csv\n")
    f.write("time_domain_centered_all_axes.csv\n")
    f.write("fft_results_all_sensor_axes.csv\n")
    f.write("fft_results_all_axes.csv\n")
    f.write("top_fft_peaks_all_sensor_axes.csv\n")
    f.write("frequency_band_summary_all_sensor_axes.csv\n")

    if transmissibility_csv_path is not None:
        f.write("transmissibility_results_all_axes.csv\n")
        f.write("transmissibility_summary.csv\n")

    if data_mode != "single_sensor_legacy":
        for sensor in SENSOR_NAMES:
            f.write(f"time_signal_all_axes_{sensor}.png\n")
            f.write(f"fft_plot_all_axes_{sensor}.png\n")

        if len(SENSOR_NAMES) > 1:
            f.write("fft_overlap_x_axis_selected_sensors.png\n")
            f.write("fft_overlap_y_axis_selected_sensors.png\n")
            f.write("fft_overlap_z_axis_selected_sensors.png\n")
    else:
        f.write("time_signal_all_axes.png\n")
        f.write("fft_plot_all_axes.png\n")

    for result in summary_results:
        f.write(f"{result['fft_csv']}\n")
        f.write(f"{result['time_plot']}\n")
        f.write(f"{result['fft_plot']}\n")

print("\nAnalysis complete.", flush=True)
print(f"\nProcessed folder created:\n{run_folder}", flush=True)
print(f"\nSummary saved to:\n{summary_txt_path}", flush=True)
print(f"\nTop peaks saved to:\n{peak_table_csv_path}", flush=True)
print(f"\nFrequency band summary saved to:\n{band_summary_csv_path}", flush=True)
print(f"\nSampling quality saved to:\n{sampling_stats_csv_path}", flush=True)

if transmissibility_csv_path is not None:
    print(f"\nTransmissibility results saved to:\n{transmissibility_csv_path}", flush=True)
