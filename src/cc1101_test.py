import spidev
import RPi.GPIO as GPIO
import time

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device (CS) 0
spi.max_speed_hz = 500000  # Set SPI speed (adjust as necessary)

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GDO2_PIN = 25  # Example GPIO pin connected to GDO2
GPIO.setup(GDO2_PIN, GPIO.IN)

def write_register(addr, value):
    spi.xfer2([addr | 0x80, value])

def read_register(addr):
    return spi.xfer2([addr & 0x7F, 0x00])[1]

# CC1101 Configuration for RAW MODE
CC1101_CONFIG = {
    0x00: 0x0D,  # GDO2 (Pin 25) outputs raw asynchronous serial data

    0x02: 0x30,  # ASK/OOK, no Manchester, no sync word requirement
    #0x02: 0x2F,  # GFSK instead of OOK

    0x06: 0x0D,  # GDO0 output: Asynchronous serial data
    #0x08: 0x00,  # PKTCTRL1: No address check
    #0x09: 0x32,  # PKTCTRL0: Disable packet processing (Raw mode)

    0x08: 0x00,  # Disable address check
    0x09: 0x00,  # Enable raw mode (no CRC, no sync)
    0x0A: 0x00,  # Infinite packet length
    

    #0x0B: 0x10,  # 58 kHz Channel bandwidth (Better for narrowband reception)
    0x10: 0x45,  # Wider bandwidth

    0x0D: 0x10,  # Frequency high byte (1093745 → 433.92 MHz)
    0x0E: 0xB0,  # Frequency middle byte
    0x0F: 0x71,  # Frequency low byte

    0x10: 0x15,  # Wider bandwidth for ASK/OOK
    0x11: 0x83,  # Symbol rate mantissa
    0x12: 0x30,  # Modem config: Asynchronous serial mode, OOK modulation
    0x15: 0x00,  # Disable frequency offset compensation

    0x18: 0x18,  # Main Radio Control State Machine configuration
    0x19: 0x1D,  # Stronger AGC settings for better reception
    0x1B: 0x07,  # Faster AGC response

    0x21: 0x56,  # Front end RX configuration
    0x22: 0x10,  # Front end RX config for OOK
    0x25: 0x00,  # Frequency synthesizer calibration
    0x26: 0x11,  # Frequency synthesizer calibration
    0x29: 0x59,  # Frequency synthesizer calibration
    0x2C: 0x81,  # Frequency synthesizer calibration
    0x2D: 0x35,  # Frequency synthesizer calibration
    0x2E: 0x09,  # Frequency synthesizer calibration
}

# Write configuration to CC1101
print("Configuring CC1101 for Raw Mode...")
for reg, value in CC1101_CONFIG.items():
    write_register(reg, value)
    print(f"x0x{reg:02X}: 0x{value:02X}")

# Send SIDLE command to ensure CC1101 is in idle state and set in receive mode
spi.xfer2([0x36])  # SIDLE
spi.xfer2([0x34])  # RX Mode

print("Listening for raw data on GDO2 (Pin 25)...")

# Main loop to listen for incoming data
try:
    while True:
        if GPIO.input(GDO2_PIN) == GPIO.HIGH:
            print("HIGH detected on GDO2")
        else:
            print("LOW detected on GDO2")
        time.sleep(0.01)  # Polling delay (10ms)
except KeyboardInterrupt:
    pass
finally:
    spi.close()
    GPIO.cleanup()