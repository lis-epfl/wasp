import smbus2
import time

# bus = smbus2.SMBus(1)
# addr = 0x57
# value = 0x01
# bus.write_byte(addr,value)
# time.sleep(0.12)
# data = bus.read_byte(addr)
# data <<= 8
# data = bus.read_byte(addr)
# data <<=8
# data = bus.read_byte(addr)

# dist = float(data)/1000

# print(dist)

addr = 0x57
i2c.writeto(addr, bytes([0x01]))
time.sleep(0.12)  # 120 milliseconds delay
data = bytearray(3)
i2c.readfrom_into(self.addr, data)
distance = (data[0] << 16 | data[1] << 8 | data[2]) / 1000.0
return min(distance, 4500.00)  # Limit to 4500 cm