import serial

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
                li550_data[config.LI550_MAPPING[tag]] = float(value)
            except ValueError:
                li550_data[config.LI550_MAPPING[tag]] = value
    return li550_data