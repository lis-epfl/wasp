import spidev
import time
import RPi.GPIO as GPIO

# --- SPI Setup ---
spi = spidev.SpiDev()
spi.open(0, 1)  # Bus 0, CE1 (SPI0.1)
spi.max_speed_hz = 500000
spi.mode = 0

# --- SPI Helpers ---
def write_register(addr, value):
    # Write: bit7 = 0
    resp = spi.xfer2([addr & 0x7F, value])
    return resp[0]  # Status byte

def read_register(addr):
    # Read: bit7 = 1
    resp = spi.xfer2([addr | 0x80, 0x00])
    return resp[1], resp[0]  # Data, Status byte

# --- CC1101 Reset ---
print("Resetting CC1101...")
spi.xfer2([0x30])  # SRES reset strobe
time.sleep(0.1)    # Mandatory delay after reset!

# --- Verify PARTNUM & VERSION ---
partnum, status = read_register(0x30)  # PARTNUM
version, _ = read_register(0x31)       # VERSION
print(f"PARTNUM: 0x{partnum:02X}, VERSION: 0x{version:02X}, STATUS: 0x{status:02X}")

# CC1101 Configuration for RAW MODE
CC1101_CONFIG = {
    0x00: 0x0D,  # IOCFG2        GDO2 Output Pin Configuration
    0x02: 0x0D,  # IOCFG0        GDO0 Output Pin Configuration
    0x03: 0x47,  # FIFOTHR       RX FIFO and TX FIFO Thresholds
    0x08: 0x32,  # PKTCTRL0      Packet Automation Control
    0x0B: 0x06,  # FSCTRL1       Frequency Synthesizer Control
    0x0D: 0x10,  # FREQ2         Frequency Control Word, High Byte
    0x0E: 0xB0,  # FREQ1         Frequency Control Word, Middle Byte
    0x0F: 0x71,  # FREQ0         Frequency Control Word, Low Byte
    0x10: 0xF6,  # MDMCFG4       Modem Configuration
    0x11: 0x83,  # MDMCFG3       Modem Configuration
    0x12: 0x30,  # MDMCFG2       Modem Configuration
    0x13: 0x20,  # MDMCFG1       Modem Configuration
    0x14: 0xF7,  # MDMCFG0       Modem Configuration
    0x15: 0x15,  # DEVIATN       Modem Deviation Setting
    0x18: 0x18,  # MCSM0         Main Radio Control State Machine Configuration
    0x19: 0x16,  # FOCCFG        Frequency Offset Compensation Configuration
    0x1B: 0xFB,  # WORCTRL       Wake On Radio Control
    0x22: 0x11,  # FREND0        Front End TX Configuration
    0x23: 0xE9,  # FSCAL3        Frequency Synthesizer Calibration
    0x24: 0x2A,  # FSCAL2        Frequency Synthesizer Calibration
    0x25: 0x00,  # FSCAL1        Frequency Synthesizer Calibration
    0x26: 0x1F,  # FSCAL0        Frequency Synthesizer Calibration
    0x2C: 0x81,  # TEST2         Various Test Settings
    0x2D: 0x35,  # TEST1         Various Test Settings
    0x2E: 0x09,  # TEST0         Various Test Settings
}

print("Configuring CC1101...")
for reg, value in CC1101_CONFIG.items():
    status_w = write_register(reg, value)
    time.sleep(0.01)
    read_val, status_r = read_register(reg)
    print(f"Reg 0x{reg:02X} → Wrote 0x{value:02X}, Read 0x{read_val:02X}, Write Status: 0x{status_w:02X}, Read Status: 0x{status_r:02X}")
print("Done!")

# Send SIDLE command to ensure CC1101 is in idle state and set in receive mode
spi.xfer2([0x36])  # SIDLE
spi.xfer2([0x34])  # RX Mode


# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GDO2_PIN = 25  # GPIO25 (Pin 22)
GPIO.setup(GDO2_PIN, GPIO.IN)

# Main loop to listen for incoming data
try:
    while True:
        if GPIO.input(GDO2_PIN) == GPIO.HIGH:
            print("1")
        else:
            print("0")
        time.sleep(0.000417)  # Polling delay = Length of each pulse = 417 µs
except KeyboardInterrupt:
    pass
finally:
    spi.close()
    GPIO.cleanup()


