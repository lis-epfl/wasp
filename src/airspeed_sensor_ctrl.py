import serial
import csv
import time

import plot_li550_data
import config

# CSV file setup
csv_path = 'li550_data.csv'
fieldnames = ['Timestamp', 'S', 'S2', 'D', 'DV', 'U', 'V', 'W', 'T', 'C', 'H', 'DP', 'P', 'AD', 'PI', 'RO', 'MD', 'TD']

# Open the serial connection
ser = serial.Serial(config.SERIAL_PORT_LI550, config.BAUD_RATE_LI550, timeout=1)

time_start_abs = time.time()  # Get the absolute start time

# Open the CSV file and write the header
with open(csv_path, mode='w', newline='') as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()

    try:
        while True:
            time_start_while = time.time()

            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue  # Skip if line is empty

            # Split the line into [Tag, Value, Tag, Value, ...]
            parts = line.split()
            timestamp = time.time() - time_start_abs
            data = {'Timestamp': timestamp}

            # Process parts two by two
            for i in range(0, len(parts) - 1, 2):
                tag = parts[i]
                value = parts[i+1]
                if tag in fieldnames:
                    try:
                        data[tag] = float(value)
                    except ValueError:
                        data[tag] = value  # If not a number, store raw

            # Write the row
            writer.writerow(data)
            csvfile.flush()

            # Sleep to respect the desired loop time
            time_end_while = time.time()
            # print("LI550 process:", time_end_while - time_start_while, "s") # 0.05s usually
            if time_end_while - time_start_while < config.DT_LI550:
                time.sleep(config.DT_LI550 - (time_end_while - time_start_while))
            else:
                print(f"Main process: Execution time exceeded {config.DT_LI550} seconds.")

    except KeyboardInterrupt:
        print("Data logging stopped.")

    finally:
        ser.close()
        print(f"\nRun complete. Data saved to {csv_path}")
        try:
            # plot_li550_data.plot_data(csv_path)
            plot_li550_data.create_video_from_data(csv_path)
        except Exception as e:
            print(f"Plotting failed: {e}")
        