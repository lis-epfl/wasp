from datetime import date
import csv
import os


def save_calibration_data(zipline_length):
    """
    Save the calibration data to a file named with the current date
    """
    # Create the calibration directory if it doesn't exist
    os.makedirs("zipline_calib", exist_ok=True)
    
    # Generate filename with current date
    today = date.today().strftime("%Y-%m-%d")
    filename = f"zipline_calib/zipline_{today}.csv"
    
    # Save the calibration data
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['zipline_length'])
        writer.writerow([zipline_length])
    
    print(f"Calibration data saved to {filename}")


def load_calibration_data():
    """
    Load calibration data from a file named with the current date if it exists
    """
    # Generate filename with current date
    today = date.today().strftime("%Y-%m-%d")
    filename = f"zipline_calib/zipline_{today}.csv"
    
    # Check if the file exists
    if not os.path.exists(filename):
        print(f"No calibration file found for today ({today})")
        return None
    
    # Load the calibration data
    try:
        with open(filename, 'r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip header
            row = next(reader)
            zipline_length = float(row[0])
            
            # Validate the data
            if zipline_length <= 0:
                print(f"Invalid zipline length in calibration file: {zipline_length}")
                return None
                
            print(f"Calibration data loaded from {filename}")
            return zipline_length
    except Exception as e:
        print(f"Error loading calibration data: {e}")
        return None