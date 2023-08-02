import board
import busio
i2c = busio.I2C(board.SCL, board.SDA)

import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
ads = ADS.ADS1115(i2c)

ads.gain = 2/3

chan = AnalogIn(ads, ADS.P0)

while True:
	value = chan.value
	voltage = chan.voltage
	print("Voltage: ", int(voltage*1000)-2000)
	print("Value: ", value)
