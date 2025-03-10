from pycc1101 import CC1101

radio = CC1101()
radio.set_frequency(433.92)  # Set correct frequency
radio.set_modulation('ASK')  # Set ASK/OOK mode
radio.disable_address_check()  # Disable filtering for now

while True:
    data = radio.receive()  # Read incoming data
    if data:
        print("Received:", data.hex())  # Print as hex