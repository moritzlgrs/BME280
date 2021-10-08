import smbus
import time
import math

class BME280:
    
    messwerte = None
    
    temp = None
    pres = None
    hum = None
    
    bus = smbus.SMBus(1)
    address = 0x76

    def reset(self):
        self.bus.write_byte_data(self.address, 0xE0, 0xB6)


    def calibration_data(self, data, i):
        value = (data[i + 1] << 8) + data[i]
        if value >= 54890:
            value -= math.pow(2,16)
        return value

    def measure(self, oversample_hum = 2, oversample_pres = 2, oversample_temp = 2, mode = 1):
        
        id_and_version = self.bus.read_i2c_block_data(self.address, 0xD0, 2)
        #print "id: ", id_and_version[0]
        #print "version: ", id_and_version[1]
    
        reg_control_hum = 0xF2
        self.bus.write_byte_data(self.address, reg_control_hum, oversample_hum)
        
        reg_control_meas = 0xF4
        oversample_meas = (oversample_temp << 5) + (oversample_pres << 2) + mode
        self.bus.write_byte_data(self.address, reg_control_meas, oversample_meas)
          
        t_and_p = self.bus.read_i2c_block_data(self.address, 0x88, 24)
    
        dig_T1 = self.calibration_data(t_and_p, 0)
        dig_T2 = self.calibration_data(t_and_p, 2)
        dig_T3 = self.calibration_data(t_and_p, 4)
    
        dig_P1 = self.calibration_data(t_and_p, 6)
        dig_P2 = self.calibration_data(t_and_p, 8)
        dig_P3 = self.calibration_data(t_and_p, 10)
        dig_P4 = self.calibration_data(t_and_p, 12)
        dig_P5 = self.calibration_data(t_and_p, 14)
        dig_P6 = self.calibration_data(t_and_p, 16)
        dig_P7 = self.calibration_data(t_and_p, 18)
        dig_P8 = self.calibration_data(t_and_p, 20)
        dig_P9 = self.calibration_data(t_and_p, 22)
    
        dig_H1 = self.bus.read_byte_data(self.address, 0xA1)
        dig_H2 = (self.bus.read_byte_data(self.address, 0xE2) << 8) + self.bus.read_byte_data(self.address, 0xE1) 
        dig_H3 = self.bus.read_byte_data(self.address, 0xE3)
        dig_H4 = (self.bus.read_byte_data(self.address, 0xE4) << 4) + (self.bus.read_byte_data(self.address, 0xE5) & 0x0F)
        dig_H5 = (self.bus.read_byte_data(self.address, 0xE6) << 4) + (self.bus.read_byte_data(self.address, 0xE5) >> 4)
        dig_H6 = self.bus.read_byte_data(self.address, 0xE7)
    
    
        wait_time = 1.25 + (2.3 * oversample_temp) + ((2.3 * oversample_pres) + 0.575) + ((2.3 * oversample_hum) + 0.575)
        time.sleep(wait_time / 1000)
    
        #get the raw measurements
        messwerte = self.bus.read_i2c_block_data(self.address, 0xF7, 8)

        self.pres_raw = (messwerte[0] << 12) + (messwerte[1] << 4) + (messwerte[2] >> 4)
        self.temp_raw = (messwerte[3] << 12) + (messwerte[4] << 4) + (messwerte[5] >> 4)
        self.hum_raw = (messwerte[6] << 8) + (messwerte[7])

    
        #self.temperature
        var1 = ((self.temp_raw) / 16384.0 - (dig_T1) / 1024.0) * (dig_T2)
        var2 = (((self.temp_raw) / 131072.0 - (dig_T1) / 8192.0) * ((self.temp_raw) / 131072.0 - (dig_T1) / 8192.0)) * (dig_T3)
        t_fine = var1+var2
        self.temp = (var1 + var2) / 5120.0
    
        #self.pres
        var1 = (t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * dig_P6 / 32768.0
        var2 = var2 + var1 * dig_P5 * 2.0
        var2 = (var2 / 4.0) + (dig_P4 * 65536.0)
        var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * dig_P1
        if var1 == 0:
            self.pres = 0
        else:
            self.pres = 1048576.0 - self.pres_raw
            self.pres = (self.pres - (var2 / 4096.0)) * 6250.0 / var1
            var1 = dig_P9 * self.pres * self.pres / 2147483648.0
            var2 = self.pres * dig_P8 / 32768.0
            self.pres = self.pres + (var1 + var2 + dig_P7) / 16.0
            self.pres /= 100
        
        #self.hum
        self.hum = (t_fine) - 76800.0
        self.hum = (self.hum_raw - ((dig_H4) * 64.0 + (dig_H5) / 16384.0 * self.hum)) * ((dig_H2) / 65536.0 * (1.0 + (dig_H6) / 67108864.0 * self.hum * (1.0 + (dig_H3) / 67108864.0 * self.hum)))
        self.hum = self.hum * (1.0 - (dig_H1) * self.hum / 524288.0)
        if self.hum > 100.0:
            self.hum = 100.0
        elif self.hum < 0.0:
            self.hum = 0


        self.hum = t_fine - 76800.0
        self.hum = (self.hum_raw - (dig_H4 * 64.0 + dig_H5 / 16384.0 * self.hum)) * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * self.hum * (1.0 + dig_H3 / 67108864.0 * self.hum)))
        self.hum = self.hum * (1.0 - dig_H1 * self.hum / 524288.0)
        if self.hum > 100:
            self.hum = 100
        elif self.hum < 0:
            self.hum = 0
        
        return self.temp, self.hum, self.pres
        #tschuesch