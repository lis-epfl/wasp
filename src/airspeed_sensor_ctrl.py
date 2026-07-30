import serial
import time

import config

def log_li550_data(ser):
    """
    Reads data from the LI-5500 sensor and writes it to a CSV file.
    :param ser: The serial connection to the LI-5500 sensor.
    """
    line = ser.readline().decode('utf-8').strip()
    if not line:
        return {}

    parts = line.split()

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


if __name__ == "__main__":
    try:
        ser = serial.Serial(config.SERIAL_PORT_LI550, config.BAUD_RATE_LI550, timeout=1)
        print(f"Connected to LI-5500 on {config.SERIAL_PORT_LI550} at {config.BAUD_RATE_LI550} bps")
        
        while True:
            data = log_li550_data(ser)
            if data:
                print(data)
            time.sleep(1)  # Adjust the sleep time as needed

    except serial.SerialException as e:
        print(f"Error connecting to LI-5500: {e}")
    finally:
        if ser.is_open:
            ser.close()