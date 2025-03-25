import spidev
import time

import config


def write_register(addr, value, spi):
    '''
    Write a value to a register in the CC1101.
    :param addr: Register address
    :param value: Value to write
    :param spi: SPI object
    :return: Status byte    
    '''
    resp = spi.xfer2([addr & 0x7F, value]) # Write: bit7 = 0
    return resp[0]  # Status byte


def read_register(addr, spi):
    '''
    Read a value from a register in the CC1101.
    :param addr: Register address
    :param spi: SPI object
    :return: Tuple of (Data, Status)
    '''
    resp = spi.xfer2([addr | 0x80, 0x00]) # Read: bit7 = 1
    return resp[1], resp[0]  # Data, Status byte


def ini_rf_reciever():
    '''
    Initialize the CC1101 as receiver to match the signal sent by the HT Keyfob transmitter.
    '''
    # --- SPI Setup ---
    spi = spidev.SpiDev()
    spi.open(0, 1)  # Open SPI bus 0, chip select 1 (= CE1 = GPIO7 = PIN26)
    spi.max_speed_hz = 500000
    spi.mode = 0

    # --- CC1101 Reset ---
    spi.xfer2([0x30])  # SRES reset strobe
    time.sleep(0.1)    # Mandatory delay after reset!

    #print("Configuring CC1101...")
    for reg, value in config.CC1101_CONFIG.items():
        status_w = write_register(reg, value, spi)
        time.sleep(0.01)
        read_val, status_r = read_register(reg, spi)
        #print(f"Reg 0x{reg:02X} → Wrote 0x{value:02X}, Read 0x{read_val:02X}, Write Status: 0x{status_w:02X}, Read Status: 0x{status_r:02X}")
    #print("Done!")

    # Send SIDLE command to ensure CC1101 is in idle state and set in receive mode
    spi.xfer2([0x36])  # SIDLE
    spi.xfer2([0x34])  # RX Mode