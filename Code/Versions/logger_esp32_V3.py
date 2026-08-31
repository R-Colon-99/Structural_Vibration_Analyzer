# V3 - App-ready logger with selectable sensor count
# Structural Vibration Analyzer

import argparse
import csv
import struct
import time
from pathlib import Path
from datetime import datetime

import serial

# ============================================================
# DEFAULT SETTINGS
# ============================================================

DEFAULT_PORT = "COM3"
DEFAULT_BAUD = 921600
DEFAULT_DATA_DIR = Path(r"F:\RLenovo99\Documents\Projects\Structural_Vibration_Analyzer\Data\raw")

ACCEL_SCALE = 4096.0  # ±8g
GYRO_SCALE = 131.0    # ±250 deg/s

SYNC = b"\xAA\x55"
PACKET_TYPE_DATA = 0x01


# ============================================================
# ARGUMENTS
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="ESP32 binary logger for the Structural Vibration Analyzer."
    )

    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"Serial port to use. Default: {DEFAULT_PORT}",
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"Serial baud rate. Default: {DEFAULT_BAUD}",
    )

    parser.add_argument(
        "--output",
        default=None,
        help="Output CSV file path. If omitted, a timestamped file is created.",
    )

    parser.add_argument(
        "--data-dir",
        default=str(DEFAULT_DATA_DIR),
        help="Folder used for timestamped output files when --output is omitted.",
    )

    parser.add_argument(
        "--num-sensors",
        type=int,
        choices=[1, 2, 3],
        default=3,
        help="Number of MPU6050 sensors expected in each binary packet. Default: 3.",
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Optional recording duration in seconds. If omitted, records until stopped.",
    )

    parser.add_argument(
        "--prompt",
        action="store_true",
        help="Ask for ENTER before recording. Useful when running manually.",
    )

    return parser.parse_args()


args = parse_args()

PORT = args.port
BAUD = args.baud
NUM_SENSORS = args.num_sensors

DATA_DIR = Path(args.data_dir)
DATA_DIR.mkdir(parents=True, exist_ok=True)

if args.output:
    output_file = Path(args.output)
    output_file.parent.mkdir(parents=True, exist_ok=True)
else:
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_file = DATA_DIR / f"esp32_vibration_{timestamp}.csv"

PAYLOAD_SIZE = 4 + 4 + (NUM_SENSORS * 6 * 2)
FRAME_SIZE = 2 + 1 + 1 + PAYLOAD_SIZE + 1


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def calculate_checksum(frame_without_checksum: bytes) -> int:
    checksum = 0
    for byte in frame_without_checksum:
        checksum ^= byte
    return checksum


def read_until_binary_start(ser: serial.Serial) -> None:
    print("\nWaiting for ESP32 startup/calibration messages...\n", flush=True)

    while True:
        line = ser.readline()

        if not line:
            continue

        text = line.decode("utf-8", errors="ignore").strip()

        if text:
            print(text, flush=True)

        if text == "START_BINARY":
            print("\nBinary stream detected.", flush=True)
            return


def read_exactly(ser: serial.Serial, number_of_bytes: int) -> bytes:
    data = bytearray()

    while len(data) < number_of_bytes:
        chunk = ser.read(number_of_bytes - len(data))

        if not chunk:
            raise TimeoutError("Serial timeout while reading binary packet.")

        data.extend(chunk)

    return bytes(data)


def read_frame(ser: serial.Serial):
    # Search for sync bytes. This lets the logger recover if it starts
    # in the middle of a binary frame or if one frame is corrupted.
    previous = b""

    while True:
        current = ser.read(1)

        if not current:
            raise TimeoutError("Serial timeout while searching for sync bytes.")

        if previous + current == SYNC:
            break

        previous = current

    header_rest = read_exactly(ser, 2)
    packet_type = header_rest[0]
    payload_size = header_rest[1]

    if packet_type != PACKET_TYPE_DATA or payload_size != PAYLOAD_SIZE:
        return None

    payload = read_exactly(ser, PAYLOAD_SIZE)
    received_checksum = read_exactly(ser, 1)[0]

    frame_without_checksum = SYNC + header_rest + payload
    expected_checksum = calculate_checksum(frame_without_checksum)

    if received_checksum != expected_checksum:
        return None

    return payload


def decode_payload(payload: bytes):
    offset = 0

    t_us, sample_index = struct.unpack_from("<II", payload, offset)
    offset += 8

    row = {
        "t_us": t_us,
        "sample_index": sample_index,
    }

    # Keep legacy sensor-1 column names so older analysis habits still work.
    legacy_values = {}

    for sensor_number in range(1, NUM_SENSORS + 1):
        raw_values = struct.unpack_from("<hhhhhh", payload, offset)
        offset += 12

        ax_g = raw_values[0] / ACCEL_SCALE
        ay_g = raw_values[1] / ACCEL_SCALE
        az_g = raw_values[2] / ACCEL_SCALE
        gx_dps = raw_values[3] / GYRO_SCALE
        gy_dps = raw_values[4] / GYRO_SCALE
        gz_dps = raw_values[5] / GYRO_SCALE
        a_resultant_g = (ax_g**2 + ay_g**2 + az_g**2) ** 0.5

        prefix = f"s{sensor_number}"

        row[f"{prefix}_ax_g"] = ax_g
        row[f"{prefix}_ay_g"] = ay_g
        row[f"{prefix}_az_g"] = az_g
        row[f"{prefix}_gx_dps"] = gx_dps
        row[f"{prefix}_gy_dps"] = gy_dps
        row[f"{prefix}_gz_dps"] = gz_dps
        row[f"{prefix}_a_resultant_g"] = a_resultant_g

        if sensor_number == 1:
            legacy_values = {
                "ax_g": ax_g,
                "ay_g": ay_g,
                "az_g": az_g,
                "gx_dps": gx_dps,
                "gy_dps": gy_dps,
                "gz_dps": gz_dps,
                "a_resultant_g": a_resultant_g,
            }

    row.update(legacy_values)
    return row


def build_fieldnames(num_sensors: int):
    fieldnames = ["t_us", "sample_index"]

    for sensor_number in range(1, num_sensors + 1):
        prefix = f"s{sensor_number}"
        fieldnames += [
            f"{prefix}_ax_g",
            f"{prefix}_ay_g",
            f"{prefix}_az_g",
            f"{prefix}_gx_dps",
            f"{prefix}_gy_dps",
            f"{prefix}_gz_dps",
            f"{prefix}_a_resultant_g",
        ]

    fieldnames += [
        "ax_g",
        "ay_g",
        "az_g",
        "gx_dps",
        "gy_dps",
        "gz_dps",
        "a_resultant_g",
    ]

    return fieldnames


# ============================================================
# STARTUP MESSAGE
# ============================================================

fieldnames = build_fieldnames(NUM_SENSORS)

print("\n========================================", flush=True)
print(" ESP32 MPU6050 BINARY VIBRATION LOGGER", flush=True)
print("========================================", flush=True)
print(f"\nPort: {PORT}", flush=True)
print(f"Baud: {BAUD}", flush=True)
print(f"Sensors expected: {NUM_SENSORS}", flush=True)
print(f"Expected payload size: {PAYLOAD_SIZE} bytes", flush=True)
print(f"Expected frame size: {FRAME_SIZE} bytes", flush=True)
print(f"Saving to:\n{output_file}", flush=True)

if args.prompt:
    input("\nPress ENTER to START recording...")

print("\nOpening serial port...", flush=True)
print("Stop from the PC app or press CTRL + C to stop and save the file.\n", flush=True)

valid_frames = 0
bad_frames = 0
start_time = time.monotonic()

try:
    with serial.Serial(PORT, BAUD, timeout=2) as ser, open(output_file, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        read_until_binary_start(ser)

        print("\nRecording binary data and saving decoded CSV...", flush=True)

        while True:
            if args.duration is not None and (time.monotonic() - start_time) >= args.duration:
                print("\nRequested duration reached.", flush=True)
                break

            payload = read_frame(ser)

            if payload is None:
                bad_frames += 1
                continue

            row = decode_payload(payload)
            writer.writerow(row)
            valid_frames += 1

            if valid_frames % 250 == 0:
                print(
                    f"Samples: {valid_frames} | "
                    f"t = {row['t_us'] / 1_000_000.0:.3f} s | "
                    f"bad frames: {bad_frames}",
                    end="\r",
                    flush=True,
                )

except KeyboardInterrupt:
    print("\n\nRecording stopped by user.", flush=True)

finally:
    print("\n\nRecording finished.", flush=True)
    print(f"Valid samples saved: {valid_frames}", flush=True)
    print(f"Bad/corrupted frames skipped: {bad_frames}", flush=True)
    print(f"\nData saved to:\n{output_file}", flush=True)
    print("\nDone.", flush=True)
