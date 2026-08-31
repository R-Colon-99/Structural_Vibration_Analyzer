# V3 - App-ready logger for Structural Vibration Analyzer

import argparse
import csv
import signal
import struct
import sys
from pathlib import Path
from datetime import datetime

import serial

# ============================================================
# DEFAULT SETTINGS
# ============================================================

DEFAULT_PORT = "COM3"
DEFAULT_BAUD = 921600
DEFAULT_DATA_DIR = Path(r"F:\RLenovo99\Documents\Projects\Structural_Vibration_Analyzer\Data\raw")

NUM_SENSORS = 3
ACCEL_SCALE = 4096.0  # ±8g
GYRO_SCALE = 131.0    # ±250 deg/s

SYNC = b"\xAA\x55"
PACKET_TYPE_DATA = 0x01
PAYLOAD_SIZE = 4 + 4 + (NUM_SENSORS * 6 * 2)
FRAME_SIZE = 2 + 1 + 1 + PAYLOAD_SIZE + 1

stop_requested = False


# ============================================================
# COMMAND-LINE ARGUMENTS
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="ESP32 3-MPU6050 binary vibration logger."
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"Serial COM port. Default: {DEFAULT_PORT}",
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
        help="Full CSV output path. If omitted, a timestamped file is created.",
    )
    parser.add_argument(
        "--output-dir",
        default=str(DEFAULT_DATA_DIR),
        help="Folder for timestamped output when --output is not provided.",
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
        help="Ask for ENTER before starting. Useful when running manually from terminal.",
    )
    return parser.parse_args()


def make_output_file(args) -> Path:
    if args.output:
        output_file = Path(args.output)
    else:
        output_dir = Path(args.output_dir)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        output_file = output_dir / f"esp32_vibration_{timestamp}.csv"

    output_file.parent.mkdir(parents=True, exist_ok=True)
    return output_file


# ============================================================
# STOP HANDLING
# ============================================================

def request_stop(signum=None, frame=None):
    global stop_requested
    stop_requested = True
    print("\nStop requested. Finishing logger safely...", flush=True)


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)

if hasattr(signal, "SIGBREAK"):
    # Handles CTRL_BREAK_EVENT sent by the Tkinter app on Windows.
    signal.signal(signal.SIGBREAK, request_stop)


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

    while not stop_requested:
        line = ser.readline()

        if not line:
            continue

        text = line.decode("utf-8", errors="ignore").strip()

        if text:
            print(text, flush=True)

        if text == "START_BINARY":
            print("\nBinary stream detected.", flush=True)
            return

    raise KeyboardInterrupt("Stop requested before binary stream started.")


def read_exactly(ser: serial.Serial, number_of_bytes: int) -> bytes:
    data = bytearray()

    while len(data) < number_of_bytes:
        if stop_requested:
            raise KeyboardInterrupt("Stop requested while reading packet.")

        chunk = ser.read(number_of_bytes - len(data))

        if not chunk:
            raise TimeoutError("Serial timeout while reading binary packet.")

        data.extend(chunk)

    return bytes(data)


def read_frame(ser: serial.Serial):
    previous = b""

    while not stop_requested:
        current = ser.read(1)

        if not current:
            raise TimeoutError("Serial timeout while searching for sync bytes.")

        if previous + current == SYNC:
            break

        previous = current

    if stop_requested:
        raise KeyboardInterrupt("Stop requested while searching for sync bytes.")

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


def build_fieldnames():
    fieldnames = ["t_us", "sample_index"]

    for sensor_number in range(1, NUM_SENSORS + 1):
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
# MAIN LOGGER
# ============================================================

def main():
    global stop_requested

    args = parse_args()
    output_file = make_output_file(args)
    fieldnames = build_fieldnames()

    print("\n========================================", flush=True)
    print(" ESP32 3-MPU6050 BINARY VIBRATION LOGGER", flush=True)
    print("========================================", flush=True)
    print(f"\nPort: {args.port}", flush=True)
    print(f"Baud: {args.baud}", flush=True)
    print(f"Saving to:\n{output_file}", flush=True)

    if args.duration is not None:
        print(f"Duration limit: {args.duration:.3f} s", flush=True)
    else:
        print("Duration limit: none; record until stopped.", flush=True)

    if args.prompt:
        input("\nPress ENTER to START recording...")

    print("\nOpening serial port...", flush=True)
    print("Use the app Stop Logger button, CTRL+C, or close the terminal to stop.\n", flush=True)

    valid_frames = 0
    bad_frames = 0
    last_t_us = None

    try:
        with serial.Serial(args.port, args.baud, timeout=2) as ser, open(output_file, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()

            read_until_binary_start(ser)

            print("\nRecording binary data and saving decoded CSV...\n", flush=True)

            while not stop_requested:
                try:
                    payload = read_frame(ser)
                except TimeoutError as e:
                    bad_frames += 1
                    print(f"\nWarning: {e}", flush=True)
                    continue

                if payload is None:
                    bad_frames += 1
                    continue

                row = decode_payload(payload)
                writer.writerow(row)
                valid_frames += 1
                last_t_us = row["t_us"]

                if valid_frames % 250 == 0:
                    f.flush()
                    print(
                        f"Samples: {valid_frames} | "
                        f"t = {row['t_us'] / 1_000_000.0:.3f} s | "
                        f"bad frames: {bad_frames}",
                        end="\r",
                        flush=True,
                    )

                if args.duration is not None and last_t_us is not None:
                    if (last_t_us / 1_000_000.0) >= args.duration:
                        print("\nDuration reached. Stopping logger.", flush=True)
                        stop_requested = True

    except KeyboardInterrupt:
        print("\n\nRecording stopped by user/app.", flush=True)
    except serial.SerialException as e:
        print(f"\nSerial error: {e}", file=sys.stderr, flush=True)
        return 1
    except Exception as e:
        print(f"\nLogger error: {e}", file=sys.stderr, flush=True)
        return 1
    finally:
        print(f"\nValid samples saved: {valid_frames}", flush=True)
        print(f"Bad/corrupted frames skipped: {bad_frames}", flush=True)
        print(f"\nData saved to:\n{output_file}", flush=True)
        print("\nDone.", flush=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
