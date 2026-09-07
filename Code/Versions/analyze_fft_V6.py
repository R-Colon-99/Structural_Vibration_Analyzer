# V6 - App-ready FFT/FRF analyzer with fixed 0-240 Hz plot range
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

DEFAULT_RAW_DATA_DIR = Path(
    r"F:\RLenovo99\Documents\Projects\Structural_Vibration_Analyzer\Data\raw"
)
DEFAULT_PROCESSED_DIR = Path(
    r"F:\RLenovo99\Documents\Projects\Structural_Vibration_Analyzer\Data\processed"
)

DEFAULT_FILE_PATTERN = "esp32_vibration_*.csv"

AXIS_SUFFIXES = ["ax_g", "ay_g", "az_g"]

AXIS_LABELS = {
    "ax_g": "X Axis",
    "ay_g": "Y Axis",
    "az_g": "Z Axis",
}

# "auto" chooses s3 when available; otherwise the highest-numbered
# selected sensor is used as the reference/input sensor.
DEFAULT_REFERENCE_SENSOR = "auto"

NUM_TOP_PEAKS = 10
PEAK_MIN_SPACING_HZ = 2.0

# Fixed display range for FFT/FRF/coherence frequency-domain plots.
# Calculations still use the full available spectrum internally.
PLOT_FREQUENCY_MIN_HZ = 0.0
PLOT_FREQUENCY_MAX_HZ = 240.0

FREQUENCY_BANDS = [
    (0, 10),
    (10, 25),
    (25, 50),
    (50, 100),
    (100, 150),
    (150, 200),
    (200, None),
]

# Default is no artificial amplitude thresholding.
DEFAULT_NOISE_THRESHOLD_G = 0.0

# If timing variation exceeds these limits, FFT/FRF calculations are
# performed on a linearly resampled uniform time grid.
DEFAULT_JITTER_STD_RATIO_LIMIT = 0.01
DEFAULT_MAX_DT_DEVIATION_RATIO = 0.05

# Orientation correction should reflect the actual physical mounting.
# Defaults are intentionally neutral to avoid silently changing phase/sign.
SENSOR_AXIS_SIGN = {
    "s1": {"ax_g": 1, "ay_g": 1, "az_g": 1},
    "s2": {"ax_g": 1, "ay_g": 1, "az_g": 1},
    "s3": {"ax_g": 1, "ay_g": 1, "az_g": 1},
}


# ============================================================
# ARGUMENTS
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "FFT/FRF analyzer for Structural Vibration Analyzer CSV files."
        )
    )

    parser.add_argument(
        "--input",
        default=None,
        help=(
            "Raw CSV file to analyze. If omitted, the script asks "
            "or uses the latest file."
        ),
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
        help=(
            "Optional compatibility filter. Analyze only the first N available "
            "logical sensors. If omitted, sensors are detected from the CSV."
        ),
    )

    parser.add_argument(
        "--reference-sensor",
        default=DEFAULT_REFERENCE_SENSOR,
        help=(
            "Reference/input sensor for FRF calculation. "
            "Use s1, s2, s3, or auto. Default: auto."
        ),
    )

    parser.add_argument(
        "--noise-threshold-g",
        type=float,
        default=DEFAULT_NOISE_THRESHOLD_G,
        help=(
            "Optional hard amplitude threshold after mean removal. "
            "Default 0.0 disables thresholding."
        ),
    )

    parser.add_argument(
        "--show-plots",
        action="store_true",
        help=(
            "Show Matplotlib windows. Not recommended when running "
            "from the PC app."
        ),
    )

    return parser.parse_args()


args = parse_args()

RAW_DATA_DIR = Path(args.raw_dir)
PROCESSED_DIR = Path(args.output_dir)
FILE_PATTERN = args.file_pattern

PROCESSED_DIR.mkdir(parents=True, exist_ok=True)


# ============================================================
# SELECT RAW CSV FILE
# ============================================================

def select_csv_file():
    if args.input:
        input_path = Path(args.input)

        if not input_path.exists():
            raise FileNotFoundError(
                f"Input CSV file not found: {input_path}"
            )

        return input_path

    csv_files = sorted(RAW_DATA_DIR.glob(FILE_PATTERN))

    if not csv_files:
        raise FileNotFoundError(
            f"No ESP32 vibration CSV files found in: {RAW_DATA_DIR}"
        )

    print("\nAvailable raw data files:\n", flush=True)

    for i, file in enumerate(csv_files, start=1):
        print(f"{i}. {file.name}", flush=True)

    choice = input(
        "\nEnter the file number to analyze, "
        "or press ENTER to use the latest file: "
    ).strip()

    if choice == "":
        return max(csv_files, key=lambda p: p.stat().st_mtime)

    try:
        selected_index = int(choice) - 1
        return csv_files[selected_index]
    except (ValueError, IndexError):
        raise ValueError(
            "Invalid selection. Please enter a valid file number."
        )


CSV_FILE = select_csv_file()

print(f"\nAnalyzing:\n{CSV_FILE}", flush=True)


# ============================================================
# CREATE UNIQUE PROCESSED RUN FOLDER
# ============================================================

raw_stem = CSV_FILE.stem
run_folder = PROCESSED_DIR / f"{raw_stem}_processed_V6"

if run_folder.exists():
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    run_folder = (
        PROCESSED_DIR / f"{raw_stem}_processed_V6_{timestamp}"
    )

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
        required_columns = [
            f"{sensor}_{axis}" for axis in AXIS_SUFFIXES
        ]

        if all(
            column in dataframe.columns
            for column in required_columns
        ):
            available.append(sensor)

    return available


available_sensors = detect_available_sensors(df)

if available_sensors:
    if args.num_sensors is not None:
        if len(available_sensors) < args.num_sensors:
            raise ValueError(
                "The CSV contains fewer sensors than requested. "
                f"Requested: {args.num_sensors}. "
                f"Available: {available_sensors}."
            )

        SENSOR_NAMES = available_sensors[:args.num_sensors]
    else:
        SENSOR_NAMES = available_sensors

    data_mode = f"{len(SENSOR_NAMES)}_sensor"

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
        "Expected s1_ax_g/s1_ay_g/s1_az_g style columns "
        "or legacy ax_g/ay_g/az_g columns."
    )

NUM_SENSORS = len(SENSOR_NAMES)

if "t_us" not in df.columns:
    raise ValueError("Missing required column in CSV file: t_us")

if "sample_index" not in df.columns:
    print(
        "\nWarning: sample_index column not found. "
        "Dropped-sample check will be skipped.",
        flush=True,
    )

# Resolve FRF reference sensor safely.
if args.reference_sensor == "auto":
    if "s3" in SENSOR_NAMES:
        REFERENCE_SENSOR = "s3"
    else:
        REFERENCE_SENSOR = SENSOR_NAMES[-1]
else:
    REFERENCE_SENSOR = args.reference_sensor

if (
    data_mode != "single_sensor_legacy"
    and REFERENCE_SENSOR not in SENSOR_NAMES
):
    print(
        f"\nWarning: requested reference sensor "
        f"{REFERENCE_SENSOR} is not present. "
        f"Using {SENSOR_NAMES[-1]} instead.",
        flush=True,
    )
    REFERENCE_SENSOR = SENSOR_NAMES[-1]

# Copy raw file into processed folder for traceability.
raw_copy_path = run_folder / "raw_data_copy.csv"
shutil.copy2(CSV_FILE, raw_copy_path)


# ============================================================
# TIME AND SAMPLING INFORMATION
# ============================================================

t_original = df["t_us"].to_numpy(dtype=np.float64) / 1_000_000.0
t_original = t_original - t_original[0]

dt_values = np.diff(t_original)

if len(dt_values) == 0:
    raise ValueError(
        "Not enough samples to analyze. "
        "The CSV needs more than one row."
    )

if np.any(dt_values <= 0):
    raise ValueError(
        "Non-increasing timestamps detected. "
        "Check logger output before FFT analysis."
    )

mean_dt = float(np.mean(dt_values))
median_dt = float(np.median(dt_values))
std_dt = float(np.std(dt_values))
min_dt = float(np.min(dt_values))
max_dt = float(np.max(dt_values))

# Median dt is more robust when an occasional sample is dropped.
fs_nominal = 1.0 / median_dt
nyquist = fs_nominal / 2.0
duration = float(t_original[-1] - t_original[0])
num_samples_original = len(t_original)

jitter_std_ratio = (
    std_dt / mean_dt if mean_dt > 0 else 0.0
)

max_dt_deviation_ratio = float(
    np.max(np.abs(dt_values - median_dt)) / median_dt
)

if "sample_index" in df.columns:
    sample_index = df["sample_index"].to_numpy()
    sample_index_diff = np.diff(sample_index)

    positive_gaps = sample_index_diff[sample_index_diff > 1]
    dropped_samples = int(
        np.sum(positive_gaps - 1)
    )

    repeated_or_reversed = int(
        np.sum(sample_index_diff <= 0)
    )
else:
    dropped_samples = None
    repeated_or_reversed = None

needs_resampling = (
    jitter_std_ratio > DEFAULT_JITTER_STD_RATIO_LIMIT
    or max_dt_deviation_ratio > DEFAULT_MAX_DT_DEVIATION_RATIO
    or (dropped_samples is not None and dropped_samples > 0)
)

if needs_resampling:
    t_fft = np.arange(
        0.0,
        t_original[-1] + 0.5 * median_dt,
        median_dt,
    )

    # Guard against one sample extending slightly past the final timestamp.
    t_fft = t_fft[t_fft <= t_original[-1]]
else:
    t_fft = t_original.copy()

fs = fs_nominal
nyquist = fs / 2.0

sampling_stats = {
    "mean_dt_s": mean_dt,
    "median_dt_s": median_dt,
    "min_dt_s": min_dt,
    "max_dt_s": max_dt,
    "std_dt_s": std_dt,
    "jitter_std_ratio": jitter_std_ratio,
    "max_dt_deviation_ratio": max_dt_deviation_ratio,
    "estimated_fs_hz": float(fs),
    "nyquist_hz": float(nyquist),
    "duration_s": duration,
    "num_samples_original": int(num_samples_original),
    "num_samples_fft_grid": int(len(t_fft)),
    "resampling_used": bool(needs_resampling),
    "dropped_samples_estimated": (
        dropped_samples
        if dropped_samples is not None
        else "N/A"
    ),
    "repeated_or_reversed_indices": (
        repeated_or_reversed
        if repeated_or_reversed is not None
        else "N/A"
    ),
}

print(f"\nData mode: {data_mode}", flush=True)
print(f"Sensors analyzed: {', '.join(SENSOR_NAMES)}", flush=True)
print(f"Reference sensor: {REFERENCE_SENSOR}", flush=True)
print(f"Estimated sampling rate: {fs:.2f} Hz", flush=True)
print(f"Nyquist frequency: {nyquist:.2f} Hz", flush=True)

if nyquist < PLOT_FREQUENCY_MAX_HZ:
    print(
        f"WARNING: Nyquist frequency ({nyquist:.2f} Hz) is below "
        f"the requested plot maximum ({PLOT_FREQUENCY_MAX_HZ:.2f} Hz). "
        "The graph will still show 0-240 Hz, but frequencies above Nyquist "
        "contain no valid FFT data.",
        flush=True,
    )
print(f"Duration: {duration:.3f} s", flush=True)
print(f"Original samples: {num_samples_original}", flush=True)
print(f"Median dt: {median_dt:.8f} s", flush=True)
print(f"Mean dt: {mean_dt:.8f} s", flush=True)
print(f"Min dt: {min_dt:.8f} s", flush=True)
print(f"Max dt: {max_dt:.8f} s", flush=True)
print(f"Std dt: {std_dt:.8f} s", flush=True)
print(f"Jitter std/mean ratio: {jitter_std_ratio:.6f}", flush=True)
print(
    f"Maximum dt deviation ratio: "
    f"{max_dt_deviation_ratio:.6f}",
    flush=True,
)

if dropped_samples is not None:
    print(
        f"Estimated dropped samples: {dropped_samples}",
        flush=True,
    )

print(
    f"Uniform resampling before FFT/FRF: "
    f"{'YES' if needs_resampling else 'NO'}",
    flush=True,
)


# ============================================================
# PLOT HELPER
# ============================================================

def finish_plot(path):
    plt.savefig(path, dpi=300, bbox_inches="tight")

    if args.show_plots:
        plt.show()

    plt.close()


# ============================================================
# SAMPLING QUALITY OUTPUT
# ============================================================

plt.figure(figsize=(12, 5))
plt.plot(t_original[1:], dt_values)
plt.xlabel("Time [s]")
plt.ylabel("Sampling interval dt [s]")
plt.title("Sampling Jitter / Time Step Variation")
plt.grid(True)

sampling_jitter_plot_path = (
    run_folder / "sampling_jitter_dt_plot.png"
)
finish_plot(sampling_jitter_plot_path)

sampling_stats_df = pd.DataFrame([sampling_stats])
sampling_stats_csv_path = (
    run_folder / "sampling_quality_summary.csv"
)
sampling_stats_df.to_csv(sampling_stats_csv_path, index=False)


# ============================================================
# SIGNAL / FFT FUNCTIONS
# ============================================================

def get_original_centered_signal(column_name):
    signal_raw = df[column_name].to_numpy(dtype=np.float64)

    if (
        "_" in column_name
        and column_name.startswith("s")
    ):
        sensor, axis = column_name.split("_", 1)
        sign = SENSOR_AXIS_SIGN.get(
            sensor, {}
        ).get(axis, 1)
        signal_raw = sign * signal_raw

    signal_centered = signal_raw - np.mean(signal_raw)

    if args.noise_threshold_g > 0:
        signal_centered[
            np.abs(signal_centered) < args.noise_threshold_g
        ] = 0.0

    return signal_centered


def get_fft_signal(column_name):
    signal_original = get_original_centered_signal(column_name)

    if needs_resampling:
        return np.interp(
            t_fft,
            t_original,
            signal_original,
        )

    return signal_original.copy()


def compute_fft(signal_centered):
    n = len(signal_centered)

    if n < 2:
        raise ValueError("Signal needs at least two samples for FFT.")

    window = np.hanning(n)
    signal_windowed = signal_centered * window

    fft_values = np.fft.rfft(signal_windowed)
    freqs = np.fft.rfftfreq(n, d=median_dt)

    # Coherent-gain correction for Hann window.
    # For a sinusoid that falls on an FFT bin, this preserves its amplitude.
    window_sum = np.sum(window)

    if window_sum == 0:
        raise ValueError("Invalid FFT window.")

    magnitude = 2.0 * np.abs(fft_values) / window_sum

    # One-sided FFT scaling: DC and Nyquist (when present) must not be doubled.
    magnitude[0] *= 0.5

    if n % 2 == 0:
        magnitude[-1] *= 0.5

    return freqs, magnitude


def find_top_peaks(
    freqs,
    magnitude,
    num_peaks=NUM_TOP_PEAKS,
    min_spacing_hz=PEAK_MIN_SPACING_HZ,
):
    if len(freqs) < 3:
        return []

    candidate_indices = []

    for i in range(1, len(magnitude) - 1):
        if (
            magnitude[i] > magnitude[i - 1]
            and magnitude[i] > magnitude[i + 1]
        ):
            candidate_indices.append(i)

    candidate_indices = sorted(
        candidate_indices,
        key=lambda i: magnitude[i],
        reverse=True,
    )

    selected_indices = []

    for idx in candidate_indices:
        freq = freqs[idx]

        if freq <= 0:
            continue

        too_close = any(
            abs(freq - freqs[selected]) < min_spacing_hz
            for selected in selected_indices
        )

        if not too_close:
            selected_indices.append(idx)

        if len(selected_indices) >= num_peaks:
            break

    peaks = []

    for rank, idx in enumerate(selected_indices, start=1):
        peaks.append(
            {
                "rank": rank,
                "frequency_hz": float(freqs[idx]),
                "magnitude_g": float(magnitude[idx]),
            }
        )

    return peaks


def summarize_frequency_bands(freqs, magnitude, column_name):
    band_rows = []
    total_energy = np.sum(magnitude**2)

    for low, high in FREQUENCY_BANDS:
        if high is None:
            mask = freqs >= low
            band_label = f"{low}+ Hz"
        else:
            mask = (freqs >= low) & (freqs < high)
            band_label = f"{low}-{high} Hz"

        band_energy = np.sum(magnitude[mask] ** 2)

        band_rms_like = (
            np.sqrt(np.mean(magnitude[mask] ** 2))
            if np.any(mask)
            else 0.0
        )

        energy_percent = (
            band_energy / total_energy * 100.0
            if total_energy > 0
            else 0.0
        )

        band_rows.append(
            {
                "column": column_name,
                "band": band_label,
                "band_energy_g2": float(band_energy),
                "band_rms_like_g": float(band_rms_like),
                "energy_percent": float(energy_percent),
            }
        )

    return band_rows


def analyze_column(column_name):
    signal_original = get_original_centered_signal(column_name)
    signal_fft = get_fft_signal(column_name)

    rms_g = np.sqrt(np.mean(signal_original**2))
    peak_g = np.max(np.abs(signal_original))
    peak_to_peak_g = (
        np.max(signal_original) - np.min(signal_original)
    )

    freqs, magnitude = compute_fft(signal_fft)

    if len(magnitude) > 1:
        peak_index = np.argmax(magnitude[1:]) + 1
    else:
        peak_index = 0

    dominant_freq = freqs[peak_index]
    dominant_mag = magnitude[peak_index]

    top_peaks = find_top_peaks(freqs, magnitude)

    band_summary = summarize_frequency_bands(
        freqs,
        magnitude,
        column_name,
    )

    fft_df = pd.DataFrame(
        {
            "Frequency_Hz": freqs,
            f"{column_name}_Magnitude_g": magnitude,
        }
    )

    fft_csv_path = (
        run_folder / f"fft_results_{column_name}.csv"
    )
    fft_df.to_csv(fft_csv_path, index=False)

    plt.figure(figsize=(12, 5))
    plt.plot(t_original, signal_original)
    plt.xlabel("Time [s]")
    plt.ylabel(f"{column_name} [g]")
    plt.title(f"Acceleration vs Time - {column_name}")
    plt.grid(True)

    time_plot_path = (
        run_folder / f"time_signal_{column_name}.png"
    )
    finish_plot(time_plot_path)

    plt.figure(figsize=(12, 5))
    plt.plot(freqs, magnitude)
    plt.xlabel("Frequency [Hz]")
    plt.ylabel("Amplitude [g]")
    plt.title(
        f"Amplitude-Corrected FFT Frequency Spectrum - "
        f"{column_name}"
    )
    plt.grid(True)
    plt.xlim(PLOT_FREQUENCY_MIN_HZ, PLOT_FREQUENCY_MAX_HZ)

    fft_plot_path = (
        run_folder / f"fft_plot_{column_name}.png"
    )
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
# H1 FRF + COHERENCE
# ============================================================

def choose_frf_segment_length(n_samples):
    """
    Choose a power-of-two segment length suitable for averaging.
    Aim for several segments while retaining useful frequency resolution.
    """
    if n_samples < 64:
        return n_samples

    target = min(2048, n_samples)

    nperseg = 1
    while nperseg * 2 <= target:
        nperseg *= 2

    # Avoid using the entire record when there is enough data for averaging.
    while nperseg > 256 and nperseg > n_samples // 2:
        nperseg //= 2

    return max(64, nperseg)


def compute_h1_frf_and_coherence(input_signal, output_signal):
    if len(input_signal) != len(output_signal):
        raise ValueError(
            "Input and output signals must have equal lengths."
        )

    n = len(input_signal)
    nperseg = choose_frf_segment_length(n)

    if nperseg < 8:
        raise ValueError(
            "Not enough samples for FRF/coherence calculation."
        )

    step = max(1, nperseg // 2)
    window = np.hanning(nperseg)

    gxx = None
    gyy = None
    gyx = None
    segment_count = 0

    for start in range(0, n - nperseg + 1, step):
        x = input_signal[start:start + nperseg]
        y = output_signal[start:start + nperseg]

        x = x - np.mean(x)
        y = y - np.mean(y)

        X = np.fft.rfft(x * window)
        Y = np.fft.rfft(y * window)

        current_gxx = X * np.conj(X)
        current_gyy = Y * np.conj(Y)
        current_gyx = Y * np.conj(X)

        if gxx is None:
            gxx = current_gxx
            gyy = current_gyy
            gyx = current_gyx
        else:
            gxx += current_gxx
            gyy += current_gyy
            gyx += current_gyx

        segment_count += 1

    if segment_count == 0:
        raise ValueError(
            "Unable to create FRF averaging segments."
        )

    gxx /= segment_count
    gyy /= segment_count
    gyx /= segment_count

    epsilon = np.finfo(float).eps

    h1 = gyx / np.maximum(np.abs(gxx), epsilon)
    coherence = (
        np.abs(gyx) ** 2
        / np.maximum(
            np.real(gxx) * np.real(gyy),
            epsilon,
        )
    )

    coherence = np.clip(
        np.real(coherence),
        0.0,
        1.0,
    )

    freqs = np.fft.rfftfreq(
        nperseg,
        d=median_dt,
    )

    return (
        freqs,
        h1,
        coherence,
        nperseg,
        segment_count,
    )


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

    print(
        f"  Dominant frequency: "
        f"{result['dominant_freq_hz']:.2f} Hz",
        flush=True,
    )
    print(
        f"  Dominant magnitude: "
        f"{result['dominant_magnitude_g']:.6f} g",
        flush=True,
    )
    print(
        f"  RMS acceleration: "
        f"{result['rms_g']:.6f} g",
        flush=True,
    )
    print(
        f"  Peak acceleration: "
        f"{result['peak_g']:.6f} g",
        flush=True,
    )

    for peak in result["top_peaks"]:
        all_peak_rows.append(
            {
                "column": column_name,
                "rank": peak["rank"],
                "frequency_hz": peak["frequency_hz"],
                "magnitude_g": peak["magnitude_g"],
            }
        )

    all_band_rows.extend(result["band_summary"])

peak_table_csv_path = (
    run_folder / "top_fft_peaks_all_sensor_axes.csv"
)
pd.DataFrame(all_peak_rows).to_csv(
    peak_table_csv_path,
    index=False,
)

band_summary_csv_path = (
    run_folder / "frequency_band_summary_all_sensor_axes.csv"
)
pd.DataFrame(all_band_rows).to_csv(
    band_summary_csv_path,
    index=False,
)


# ============================================================
# SAVE COMBINED FFT CSV
# ============================================================

combined_fft_df = None

for column_name in columns_to_analyze:
    signal_fft = get_fft_signal(column_name)
    freqs, magnitude = compute_fft(signal_fft)

    if combined_fft_df is None:
        combined_fft_df = pd.DataFrame(
            {"Frequency_Hz": freqs}
        )

    combined_fft_df[
        f"{column_name}_Magnitude_g"
    ] = magnitude

combined_fft_csv_path = (
    run_folder / "fft_results_all_sensor_axes.csv"
)
combined_fft_df.to_csv(
    combined_fft_csv_path,
    index=False,
)

legacy_fft_csv_path = (
    run_folder / "fft_results_all_axes.csv"
)
combined_fft_df.to_csv(
    legacy_fft_csv_path,
    index=False,
)


# ============================================================
# SAVE COMBINED TIME-DOMAIN CSV
# ============================================================

time_domain_data = {"Time_s": t_original}

for column_name in columns_to_analyze:
    time_domain_data[
        f"{column_name}_centered"
    ] = get_original_centered_signal(column_name)

time_domain_df = pd.DataFrame(time_domain_data)

time_domain_csv_path = (
    run_folder / "time_domain_centered_all_sensor_axes.csv"
)
time_domain_df.to_csv(
    time_domain_csv_path,
    index=False,
)

legacy_time_csv_path = (
    run_folder / "time_domain_centered_all_axes.csv"
)
time_domain_df.to_csv(
    legacy_time_csv_path,
    index=False,
)


# ============================================================
# H1 FRF / COHERENCE ANALYSIS
# ============================================================

frf_rows = []
frf_df = None
frf_csv_path = None
frf_summary_csv_path = None

RESPONSE_SENSORS = [
    sensor
    for sensor in SENSOR_NAMES
    if sensor != REFERENCE_SENSOR
]

if (
    len(SENSOR_NAMES) >= 2
    and REFERENCE_SENSOR in SENSOR_NAMES
    and data_mode != "single_sensor_legacy"
):
    print(
        f"\nCalculating H1 acceleration FRF and coherence "
        f"using {REFERENCE_SENSOR} as reference/input sensor...",
        flush=True,
    )

    frf_frequency_base = None
    frf_df = None

    for axis in AXIS_SUFFIXES:
        reference_column = f"{REFERENCE_SENSOR}_{axis}"

        reference_signal = get_fft_signal(reference_column)

        axis_plot_data = []

        for response_sensor in RESPONSE_SENSORS:
            response_column = f"{response_sensor}_{axis}"
            response_signal = get_fft_signal(response_column)

            (
                frf_freqs,
                h1,
                coherence,
                nperseg,
                segment_count,
            ) = compute_h1_frf_and_coherence(
                reference_signal,
                response_signal,
            )

            h1_mag = np.abs(h1)
            h1_phase_deg = np.angle(
                h1,
                deg=True,
            )

            if frf_df is None:
                frf_frequency_base = frf_freqs
                frf_df = pd.DataFrame(
                    {"Frequency_Hz": frf_freqs}
                )
            else:
                if not np.allclose(
                    frf_frequency_base,
                    frf_freqs,
                ):
                    raise ValueError(
                        "FRF frequency grids do not match."
                    )

            base_name = (
                f"H1_{response_sensor}_over_"
                f"{REFERENCE_SENSOR}_{axis}"
            )

            frf_df[f"{base_name}_Magnitude"] = h1_mag
            frf_df[f"{base_name}_Phase_deg"] = h1_phase_deg
            frf_df[f"{base_name}_Coherence"] = coherence

            valid_mask = frf_freqs > 0

            if np.any(valid_mask):
                valid_freqs = frf_freqs[valid_mask]
                valid_h1_mag = h1_mag[valid_mask]
                valid_coherence = coherence[valid_mask]

                max_index = np.argmax(valid_h1_mag)

                max_h1 = valid_h1_mag[max_index]
                max_h1_freq = valid_freqs[max_index]

                mean_coherence = np.mean(valid_coherence)
                median_coherence = np.median(valid_coherence)
            else:
                max_h1 = 0.0
                max_h1_freq = 0.0
                mean_coherence = 0.0
                median_coherence = 0.0

            frf_rows.append(
                {
                    "response_sensor": response_sensor,
                    "reference_sensor": REFERENCE_SENSOR,
                    "axis": axis,
                    "max_h1_magnitude": float(max_h1),
                    "frequency_at_max_h1_hz": float(max_h1_freq),
                    "mean_coherence": float(mean_coherence),
                    "median_coherence": float(median_coherence),
                    "frf_segment_length_samples": int(nperseg),
                    "frf_averaged_segments": int(segment_count),
                }
            )

            axis_plot_data.append(
                (
                    response_sensor,
                    frf_freqs,
                    h1_mag,
                    coherence,
                )
            )

        # FRF magnitude plot.
        plt.figure(figsize=(12, 6))

        for (
            response_sensor,
            frf_freqs,
            h1_mag,
            _,
        ) in axis_plot_data:
            plt.plot(
                frf_freqs,
                h1_mag,
                label=f"{response_sensor}/{REFERENCE_SENSOR}",
            )

        plt.xlabel("Frequency [Hz]")
        plt.ylabel("H1 FRF Magnitude [-]")
        plt.title(
            f"H1 Acceleration FRF - "
            f"{AXIS_LABELS[axis]} - "
            f"Reference: {REFERENCE_SENSOR}"
        )
        plt.grid(True)
        plt.legend()
        plt.xlim(PLOT_FREQUENCY_MIN_HZ, PLOT_FREQUENCY_MAX_HZ)

        axis_letter = axis.split("_")[0][-1]

        frf_plot_path = (
            run_folder
            / f"transmissibility_{axis_letter}_axis_reference_"
            f"{REFERENCE_SENSOR}.png"
        )
        finish_plot(frf_plot_path)

        # Coherence plot.
        plt.figure(figsize=(12, 6))

        for (
            response_sensor,
            frf_freqs,
            _,
            coherence,
        ) in axis_plot_data:
            plt.plot(
                frf_freqs,
                coherence,
                label=f"{response_sensor}/{REFERENCE_SENSOR}",
            )

        plt.xlabel("Frequency [Hz]")
        plt.ylabel("Coherence [-]")
        plt.title(
            f"FRF Coherence - "
            f"{AXIS_LABELS[axis]} - "
            f"Reference: {REFERENCE_SENSOR}"
        )
        plt.grid(True)
        plt.legend()
        plt.ylim(0, 1.05)
        plt.xlim(PLOT_FREQUENCY_MIN_HZ, PLOT_FREQUENCY_MAX_HZ)

        coherence_plot_path = (
            run_folder
            / f"coherence_{axis_letter}_axis_reference_"
            f"{REFERENCE_SENSOR}.png"
        )
        finish_plot(coherence_plot_path)

    frf_csv_path = (
        run_folder / "transmissibility_results_all_axes.csv"
    )
    frf_df.to_csv(
        frf_csv_path,
        index=False,
    )

    frf_summary_csv_path = (
        run_folder / "transmissibility_summary.csv"
    )
    pd.DataFrame(frf_rows).to_csv(
        frf_summary_csv_path,
        index=False,
    )

else:
    print(
        "\nFRF/coherence analysis skipped. "
        "It requires at least 2 selected sensors.",
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

            plt.plot(
                t_original,
                get_original_centered_signal(column_name),
                label=axis,
            )

        plt.xlabel("Time [s]")
        plt.ylabel("Acceleration [g]")
        plt.title(
            f"Acceleration vs Time - All Axes - {sensor}"
        )
        plt.grid(True)
        plt.legend()

        plot_path = (
            run_folder / f"time_signal_all_axes_{sensor}.png"
        )
        finish_plot(plot_path)

else:
    plt.figure(figsize=(12, 6))

    for axis in AXIS_SUFFIXES:
        plt.plot(
            t_original,
            get_original_centered_signal(axis),
            label=axis,
        )

    plt.xlabel("Time [s]")
    plt.ylabel("Acceleration [g]")
    plt.title("Acceleration vs Time - All Axes")
    plt.grid(True)
    plt.legend()

    plot_path = (
        run_folder / "time_signal_all_axes.png"
    )
    finish_plot(plot_path)


# ============================================================
# SAVE COMBINED FFT PLOTS
# ============================================================

if data_mode != "single_sensor_legacy":
    if len(SENSOR_NAMES) > 1:
        for axis in AXIS_SUFFIXES:
            plt.figure(figsize=(12, 6))

            for sensor in SENSOR_NAMES:
                column_name = f"{sensor}_{axis}"

                plt.plot(
                    combined_fft_df["Frequency_Hz"],
                    combined_fft_df[
                        f"{column_name}_Magnitude_g"
                    ],
                    label=sensor,
                )

            plt.xlabel("Frequency [Hz]")
            plt.ylabel("Amplitude [g]")
            plt.title(
                f"Overlapped Amplitude-Corrected FFT - "
                f"{AXIS_LABELS[axis]} - Selected Sensors"
            )
            plt.grid(True)
            plt.legend()
            plt.xlim(PLOT_FREQUENCY_MIN_HZ, PLOT_FREQUENCY_MAX_HZ)

            axis_letter = axis.split("_")[0][-1]

            plot_path = (
                run_folder
                / f"fft_overlap_{axis_letter}_axis_"
                f"selected_sensors.png"
            )
            finish_plot(plot_path)

    for sensor in SENSOR_NAMES:
        plt.figure(figsize=(12, 6))

        for axis in AXIS_SUFFIXES:
            column_name = f"{sensor}_{axis}"

            plt.plot(
                combined_fft_df["Frequency_Hz"],
                combined_fft_df[
                    f"{column_name}_Magnitude_g"
                ],
                label=axis,
            )

        plt.xlabel("Frequency [Hz]")
        plt.ylabel("Amplitude [g]")
        plt.title(
            f"Amplitude-Corrected FFT - All Axes - {sensor}"
        )
        plt.grid(True)
        plt.legend()
        plt.xlim(PLOT_FREQUENCY_MIN_HZ, PLOT_FREQUENCY_MAX_HZ)

        plot_path = (
            run_folder / f"fft_plot_all_axes_{sensor}.png"
        )
        finish_plot(plot_path)

else:
    plt.figure(figsize=(12, 6))

    for axis in AXIS_SUFFIXES:
        plt.plot(
            combined_fft_df["Frequency_Hz"],
            combined_fft_df[
                f"{axis}_Magnitude_g"
            ],
            label=axis,
        )

    plt.xlabel("Frequency [Hz]")
    plt.ylabel("Amplitude [g]")
    plt.title("Amplitude-Corrected FFT - All Axes")
    plt.grid(True)
    plt.legend()
    plt.xlim(PLOT_FREQUENCY_MIN_HZ, PLOT_FREQUENCY_MAX_HZ)

    plot_path = (
        run_folder / "fft_plot_all_axes.png"
    )
    finish_plot(plot_path)


# ============================================================
# SAVE SUMMARY FILE
# ============================================================

summary_txt_path = run_folder / "summary_V6.txt"

with open(summary_txt_path, "w") as f:
    f.write(
        "STRUCTURAL VIBRATION ANALYSIS SUMMARY - V6\n"
    )
    f.write(
        "=========================================\n\n"
    )

    f.write(f"Raw file analyzed: {CSV_FILE.name}\n")
    f.write(f"Raw file path: {CSV_FILE}\n")
    f.write(f"Processed folder: {run_folder}\n")
    f.write(f"Data mode: {data_mode}\n")
    f.write(
        f"Sensors analyzed: {', '.join(SENSOR_NAMES)}\n"
    )
    f.write(f"Reference sensor: {REFERENCE_SENSOR}\n")
    f.write(
        f"Noise threshold: {args.noise_threshold_g:.6f} g\n\n"
    )

    f.write("Sampling Quality:\n")
    f.write("-----------------\n")
    f.write(
        f"Estimated sampling rate: {fs:.2f} Hz\n"
    )
    f.write(
        f"Median sampling interval: {median_dt:.8f} s\n"
    )
    f.write(
        f"Mean sampling interval: {mean_dt:.8f} s\n"
    )
    f.write(
        f"Minimum sampling interval: {min_dt:.8f} s\n"
    )
    f.write(
        f"Maximum sampling interval: {max_dt:.8f} s\n"
    )
    f.write(
        f"Sampling interval standard deviation: "
        f"{std_dt:.8f} s\n"
    )
    f.write(
        f"Jitter std/mean ratio: "
        f"{jitter_std_ratio:.8f}\n"
    )
    f.write(
        f"Maximum dt deviation ratio: "
        f"{max_dt_deviation_ratio:.8f}\n"
    )
    f.write(f"Duration: {duration:.3f} s\n")
    f.write(
        f"Original number of samples: "
        f"{num_samples_original}\n"
    )
    f.write(
        f"FFT-grid samples: {len(t_fft)}\n"
    )
    f.write(
        f"Uniform resampling used: "
        f"{'YES' if needs_resampling else 'NO'}\n"
    )
    f.write(
        f"Nyquist frequency: {nyquist:.2f} Hz\n"
    )
    f.write(
        f"Frequency-domain plot range: "
        f"{PLOT_FREQUENCY_MIN_HZ:.0f}-"
        f"{PLOT_FREQUENCY_MAX_HZ:.0f} Hz\n"
    )
    f.write(
        f"Estimated dropped samples: "
        f"{sampling_stats['dropped_samples_estimated']}\n"
    )
    f.write(
        f"Repeated or reversed sample indices: "
        f"{sampling_stats['repeated_or_reversed_indices']}\n\n"
    )

    f.write("Sensor-Axis Results:\n")
    f.write("--------------------\n\n")

    for result in summary_results:
        f.write(f"{result['column']}:\n")
        f.write(
            f"  Dominant frequency: "
            f"{result['dominant_freq_hz']:.2f} Hz\n"
        )
        f.write(
            f"  Dominant amplitude: "
            f"{result['dominant_magnitude_g']:.6f} g\n"
        )
        f.write(
            f"  RMS acceleration: "
            f"{result['rms_g']:.6f} g\n"
        )
        f.write(
            f"  Peak acceleration: "
            f"{result['peak_g']:.6f} g\n"
        )
        f.write(
            f"  Peak-to-peak acceleration: "
            f"{result['peak_to_peak_g']:.6f} g\n"
        )
        f.write(f"  FFT CSV: {result['fft_csv']}\n")
        f.write(
            f"  Time plot: {result['time_plot']}\n"
        )
        f.write(
            f"  FFT plot: {result['fft_plot']}\n"
        )
        f.write("  Top FFT peaks:\n")

        for peak in result["top_peaks"]:
            f.write(
                f"    {peak['rank']}. "
                f"{peak['frequency_hz']:.2f} Hz, "
                f"{peak['magnitude_g']:.6f} g\n"
            )

        f.write("\n")

    if frf_rows:
        f.write("H1 FRF / Coherence Summary:\n")
        f.write("---------------------------\n")
        f.write(
            f"Reference/input sensor: "
            f"{REFERENCE_SENSOR}\n"
        )
        f.write(
            f"Response sensors: "
            f"{', '.join(RESPONSE_SENSORS)}\n\n"
        )

        for row in frf_rows:
            f.write(
                f"{row['response_sensor']}/"
                f"{row['reference_sensor']} - "
                f"{row['axis']}: "
                f"Max |H1| = "
                f"{row['max_h1_magnitude']:.4f} "
                f"at "
                f"{row['frequency_at_max_h1_hz']:.2f} Hz, "
                f"Mean coherence = "
                f"{row['mean_coherence']:.4f}, "
                f"Median coherence = "
                f"{row['median_coherence']:.4f}, "
                f"Segments = "
                f"{row['frf_averaged_segments']}\n"
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

    if frf_csv_path is not None:
        f.write("transmissibility_results_all_axes.csv\n")
        f.write("transmissibility_summary.csv\n")

        for axis in AXIS_SUFFIXES:
            axis_letter = axis.split("_")[0][-1]
            f.write(
                f"coherence_{axis_letter}_axis_reference_"
                f"{REFERENCE_SENSOR}.png\n"
            )

    if data_mode != "single_sensor_legacy":
        for sensor in SENSOR_NAMES:
            f.write(
                f"time_signal_all_axes_{sensor}.png\n"
            )
            f.write(
                f"fft_plot_all_axes_{sensor}.png\n"
            )

        if len(SENSOR_NAMES) > 1:
            f.write(
                "fft_overlap_x_axis_selected_sensors.png\n"
            )
            f.write(
                "fft_overlap_y_axis_selected_sensors.png\n"
            )
            f.write(
                "fft_overlap_z_axis_selected_sensors.png\n"
            )
    else:
        f.write("time_signal_all_axes.png\n")
        f.write("fft_plot_all_axes.png\n")

    for result in summary_results:
        f.write(f"{result['fft_csv']}\n")
        f.write(f"{result['time_plot']}\n")
        f.write(f"{result['fft_plot']}\n")


print("\nAnalysis complete.", flush=True)
print(
    f"\nProcessed folder created:\n{run_folder}",
    flush=True,
)
print(
    f"\nSummary saved to:\n{summary_txt_path}",
    flush=True,
)
print(
    f"\nTop peaks saved to:\n{peak_table_csv_path}",
    flush=True,
)
print(
    f"\nFrequency band summary saved to:\n"
    f"{band_summary_csv_path}",
    flush=True,
)
print(
    f"\nSampling quality saved to:\n"
    f"{sampling_stats_csv_path}",
    flush=True,
)

if frf_csv_path is not None:
    print(
        f"\nH1 FRF/coherence results saved to:\n"
        f"{frf_csv_path}",
        flush=True,
    )
