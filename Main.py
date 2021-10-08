from BME280 import BME280

sensor = BME280()
temp, hum, pres = sensor.measure()
print(temp, hum, pres)