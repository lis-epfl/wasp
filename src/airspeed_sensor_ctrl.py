from pathlib import Path
import serial
import time

import config

# Bytes read but not yet forming a complete line, carried over between calls
# (module-level since log_li550_data is called repeatedly on the same
# open connection -- see log_li550_data's docstring for why this exists).
_leftover = b""


def log_li550_data(ser):
    """
    Reads the most recent complete line the LI-5500 has sent, discarding any
    older lines still sitting in the OS's serial input buffer.

    This used to be a single blocking ser.readline() per call. That reads
    exactly one line per call regardless of how many the sensor produced in
    the meantime, so whenever the sensor's outputrate is faster than the
    caller's read cadence (config.DT_WIND) -- or a loop iteration simply
    runs long for any other reason (CSV flush, GC pause, CPU contention from
    other processes) -- unread lines pile up in the buffer. Once that buffer
    fills, the OS driver drops/overwrites bytes wherever it happens to be
    mid-line, corrupting line framing (a tag going missing, a value turning
    into garbage) -- this is what was producing dropouts/outliers baud rate
    and outputrate mismatches had already been ruled out for.

    Draining everything currently buffered and keeping only the newest
    complete line fixes this regardless of any rate mismatch: the read
    cadence can never fall far behind, since a backlog gets fully drained
    every call rather than one line consumed per call.

    Deliberately NOT using ser.in_waiting to decide how much to read: on
    some USB-serial adapters (observed on macOS, e.g. common with CH340-
    based cables) it's unreliable and can under-report -- including reading
    back 0 even while bytes are actually arriving, which silently starves
    this function forever. ser.read(n) reads directly from the OS buffer
    regardless of what in_waiting claims, so it isn't affected by that.
    Pacing is governed by the serial connection's own (short) timeout, not
    by the caller's loop sleep alone -- see where ser is constructed.
    :param ser: The serial connection to the LI-5500 sensor.
    """
    global _leftover
    buffered = _leftover + ser.read(4096)
    lines = buffered.split(b'\n')
    _leftover = lines.pop()  # last chunk may not be newline-terminated yet

    line = b""
    for candidate in reversed(lines):
        if candidate.strip():
            line = candidate
            break
    if not line:
        return {}

    try:
        decoded = line.decode('utf-8').strip()
    except UnicodeDecodeError:
        return {}  # garbled line (e.g. a genuine baud/framing glitch) -- skip it, don't crash the process
    if not decoded:
        return {}

    parts = decoded.split()

    li550_data = {}
    for i in range(0, len(parts) - 1, 2):
        tag = parts[i]
        value = parts[i + 1]
        if tag in config.LI550_MAPPING:
            try:
                li550_data[config.LI550_MAPPING[tag]] = round(float(value), 3)
            except ValueError:
                li550_data[config.LI550_MAPPING[tag]] = value
    return li550_data


def record_and_plot(port, baud, save_dir):
    """
    Single-process standalone equivalent of main.py/main_wasp.py's
    wind_sensor_reading: same 10 Hz (config.DT_WIND) read/save cadence, same
    once-a-second CSV flush, same plot_li550_data.plot_data() call at the
    end -- just without the other processes (vision, motors, ...) main.py
    normally runs alongside it, for isolating the sensor on its own.
    Also prints U/V/W live each cycle, which the real pipeline doesn't do.
    Stops on Ctrl+C.
    """
    from datetime import datetime
    import csv
    import numpy as np
    import plot_li550_data

    save_dir = Path(save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    csv_path = save_dir / f"wind_data_{timestamp}.csv"
    csv_file = open(csv_path, 'w', newline='')
    writer = csv.DictWriter(csv_file, fieldnames=config.CSV_WIND_COLUMNS)
    writer.writeheader()

    FLUSH_INTERVAL_S = 1.0  # flush ~once a second, not every row -- see wind_sensor_reading's comment for why
    last_flush = time.time()

    ser = None
    try:
        ser = serial.Serial(port, baud, timeout=0.05)  # short: log_li550_data's read(4096) shouldn't block the loop for long
        print(f"Connected to LI-5500 on {port} at {baud} bps")
        print(f"Saving to {csv_path}")

        # Same read-then-sleep-the-remainder cadence as main.py's real
        # acquisition loop (config.DT_WIND, 10 Hz), so what you see/save here
        # is what would actually get read/saved during a real run.
        while True:
            loop_start = time.time()

            data = log_li550_data(ser)
            if data:
                uvw = {k: v for k, v in data.items() if k in ("U Vector [m/s]", "V Vector [m/s]", "W Vector [m/s]")}
                print(uvw)

            row = {'Unix Timestamp [s]': np.round(time.time(), 3), **data}
            writer.writerow(row)
            if loop_start - last_flush >= FLUSH_INTERVAL_S:
                csv_file.flush()
                last_flush = loop_start

            elapsed = time.time() - loop_start
            if elapsed < config.DT_WIND:
                time.sleep(config.DT_WIND - elapsed)
            else:
                print(f"Loop exceeded DT_WIND: {elapsed:.4f} / {config.DT_WIND:.4f} s.")

    except serial.SerialException as e:
        print(f"Error connecting to LI-5500: {e}")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if ser is not None and ser.is_open:
            ser.close()
        csv_file.close()
        print(f"\nRun complete. Wind data saved to {csv_path}")
        try:
            plot_li550_data.plot_data(csv_path)
            print("Plotting wind data complete")
        except Exception as e:
            print(f"Plotting failed: {e}")


if __name__ == "__main__":
    import sys

    # Optional port/baud/save-dir override for one-off standalone runs (e.g.
    # testing from a laptop over a USB-serial adapter, whose device path
    # won't match the Pi's config.SERIAL_PORT_LI550) without touching the
    # shared config used by the real acquisition pipeline (main.py).
    # Usage: python3 airspeed_sensor_ctrl.py [port [baud [save_dir]]]
    _port = sys.argv[1] if len(sys.argv) > 1 else config.SERIAL_PORT_LI550
    _baud = int(sys.argv[2]) if len(sys.argv) > 2 else config.BAUD_RATE_LI550
    _save_dir = sys.argv[3] if len(sys.argv) > 3 else Path(__file__).resolve().parent.parent / "data" / "02_09_with_100_45deg"

    record_and_plot(_port, _baud, _save_dir)