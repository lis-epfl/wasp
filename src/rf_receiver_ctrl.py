import spidev
import time
from collections import deque
import gpiod


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
    0x10: 0x76,  # MDMCFG4       Modem Config: BW ≈ 58 kHz, DRATE_E = 6
    0x11: 0x83,  # MDMCFG3       DRATE_M = 131 (for 2.4 kbps)
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


def ini_rf_reciever():
    global CC1101_CONFIG
    '''
    Initialize the CC1101 as receiver to match the signal sent by the HT Keyfob transmitter.
    '''
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
    spi.xfer2([0x30])  # SRES reset strobe
    time.sleep(0.1)    # Mandatory delay after reset!


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



# --------------------------------------------------------------------------------------------------------------------------------------------------

ini_rf_reciever()

GDO2_PIN = 25                 # GPIO25 (Pin 22)
BIT_DURATION = 416.7          # duration of each bit at its state in microseconds
SEQUENCE_SIZE = 121           # sequence length in bits
TIMEOUT_THRESHOLD_US = 30000  # 3000 ms 
MAX_BIT_COUNT = 40

# Butons sequences
sequences = [
    "1011001011001011011011011011011011011011011011011011011011011011011011011001001001011001011001011001011001011001011001011",
    "1011001011001011011011011011011011011011011011011011011011011011011011011001011001001001011001011001011001011001011001011",
    "1011001011001011011011011011011011011011011011011011011011011011011011011001011001011001001001011001011001011001011001011",
    "1011001011001011011011011011011011011011011011011011011011011011011011011001011001011001011001001001011001011001011001011",
    "1011001011001011011011011011011011011011011011011011011011011011011011011001011001011001011001011001001001011001011001011"
]

last_tick_ns = None
last_level = None
buffer = deque(maxlen=SEQUENCE_SIZE)
last_tick_ns = None
last_event_type = None
bit_count = 0
waiting_for_rising_edge = False

chip = gpiod.Chip('gpiochip0')
line = chip.get_line(GDO2_PIN)
line.request(consumer='sequence_detector', type=gpiod.LINE_REQ_EV_BOTH_EDGES)

last_edge_time_ns = time.time_ns()

try:
    while True:
        event = line.event_wait(sec=1)
        current_time_ns = time.time_ns()
        
        # Check for idle timeout
        if current_time_ns - last_edge_time_ns > TIMEOUT_THRESHOLD_US * 1000:
            buffer.clear()
            last_tick_ns = None
            last_event_type = None
            last_level = None
            waiting_for_rising_edge = True
            #ini_rf_reciever() # Reset CC1101
            #print("[Timeout] Buffer reset!")
        
        if event:
            evt = line.event_read()
            last_edge_time_ns = time.time_ns()
            
            # Ignore until first rising edge after timeout
            if waiting_for_rising_edge:
                if evt.type == gpiod.LineEvent.RISING_EDGE:
                    #print("[Sync] Detected rising edge, starting fresh")
                    waiting_for_rising_edge = False
                    last_tick_ns = evt.sec * 1_000_000_000 + evt.nsec
                    last_event_type = evt.type
                    last_level = 1
                continue  # Skip further processing until resynchronized
            
            # Normal processing after sync
            level = 1 if evt.type == gpiod.LineEvent.RISING_EDGE else 0
            current_tick_ns = evt.sec * 1_000_000_000 + evt.nsec

            # Check edge type change
            if last_tick_ns is not None and evt.type != last_event_type:
                delta_ns = current_tick_ns - last_tick_ns
                delta_us = delta_ns / 1000
                bit_count = round(delta_us / BIT_DURATION)

                if bit_count > 0:
                    bits_to_add = str(last_level) * bit_count
                    buffer.extend(bits_to_add)
                    #print(bits_to_add, end='', flush=True) # Print all bits detected

            # Update last tick, level, event type
            last_tick_ns = current_tick_ns
            last_level = level
            last_event_type = evt.type

            # Sequence detection
            if len(buffer) >= SEQUENCE_SIZE:
                current_seq = ''.join(buffer)
                for idx, seq in enumerate(sequences, 1):
                    if current_seq == seq:
                        print(f"\nButton {idx} detected!")

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    line.release()
    chip.close()