# labplus mPython library
# MIT license; Copyright (c) 2018 labplus
# V1.0 Zhang KaiHua(apple_eat@126.com)

# mpython buildin periphers drivers

# history:
# V1.1 add oled draw function,add buzz.freq().  by tangliufeng
# V1.2 add servo/ui class,by tangliufeng

from machine import I2C, PWM, Pin, ADC, TouchPad, UART
from ssd1306 import SSD1306_I2C
import esp, math, time, network,audio
import ustruct, array, ujson
from neopixel import NeoPixel
from esp import dht_readinto
from time import sleep_ms, sleep_us, sleep
from framebuf import FrameBuffer
import motion_mpu6050

i2c = I2C(0, scl=Pin(Pin.P19), sda=Pin(Pin.P20), freq=400000)


# class Font(object):
#     def __init__(self, font_address=0x400000):
#         self.font_address = font_address
#         buffer = bytearray(18)
#         esp.flash_read(self.font_address, buffer)
#         self.header, \
#             self.height, \
#             self.width, \
#             self.baseline, \
#             self.x_height, \
#             self.Y_height, \
#             self.first_char,\
#             self.last_char = ustruct.unpack('4sHHHHHHH', buffer)
#         self.first_char_info_address = self.font_address + 18

#     def GetCharacterData(self, c):
#         uni = ord(c)
#         # if uni not in range(self.first_char, self.last_char):
#         #     return None
#         if (uni < self.first_char or uni > self.last_char):
#             return None
#         char_info_address = self.first_char_info_address + \
#             (uni - self.first_char) * 6
#         buffer = bytearray(6)
#         esp.flash_read(char_info_address, buffer)
#         ptr_char_data, len = ustruct.unpack('IH', buffer)
#         if (ptr_char_data) == 0 or (len == 0):
#             return None
#         buffer = bytearray(len)
#         esp.flash_read(ptr_char_data + self.font_address, buffer)
#         return buffer


# class TextMode():
#     normal = 1
#     rev = 2
#     trans = 3
#     xor = 4


# class OLED(SSD1306_I2C):
#     """ 128x64 oled display """

#     def __init__(self):
#         super().__init__(128, 64, i2c)
#         self.f = Font()
#         if self.f is None:
#             raise Exception('font load failed')

#     def DispChar(self, s, x, y, mode=TextMode.normal):
#         if self.f is None:
#             return
#         for c in s:
#             data = self.f.GetCharacterData(c)
#             if data is None:
#                 x = x + self.width
#                 continue
#             width, bytes_per_line = ustruct.unpack('HH', data[:4])
#             # print('character [%d]: width = %d, bytes_per_line = %d' % (ord(c)
#             # , width, bytes_per_line))
#             for h in range(0, self.f.height):
#                 w = 0
#                 i = 0
#                 while w < width:
#                     mask = data[4 + h * bytes_per_line + i]
#                     if (width - w) >= 8:
#                         n = 8
#                     else:
#                         n = width - w
#                     py = y + h
#                     page = py >> 3
#                     bit = 0x80 >> (py % 8)
#                     for p in range(0, n):
#                         px = x + w + p
#                         c = 0
#                         if (mask & 0x80) != 0:
#                             if mode == TextMode.normal or \
#                                mode == TextMode.trans:
#                                 c = 1
#                             if mode == TextMode.rev:
#                                 c = 0
#                             if mode == TextMode.xor:
#                                 c = self.buffer[page * 128 + px] & bit
#                                 if c != 0:
#                                     c = 0
#                                 else:
#                                     c = 1
#                                 # print("px = %d, py = %d, c = %d" % (px, py, c))
#                             super().pixel(px, py, c)
#                         else:
#                             if mode == TextMode.normal:
#                                 c = 0
#                                 super().pixel(px, py, c)
#                             if mode == TextMode.rev:
#                                 c = 1
#                                 super().pixel(px, py, c)
#                         mask = mask << 1
#                     w = w + 8
#                     i = i + 1
#             x = x + width + 1

# class Accelerometer():

#     def __init__(self):
#         motion_mpu6050.init(i2c)
#         while not motion_mpu6050.accel():
#             pass
        
#     def get_x(self):
#         return -motion_mpu6050.accel()[1]/65536

#     def get_y(self):
#         return motion_mpu6050.accel()[0]/65536

#     def get_z(self):
#         return -motion_mpu6050.accel()[2]/65536

# class Motion():

#     def __init__(self):
#         motion_mpu6050.init(i2c)
#         while not motion_mpu6050.accel():
#             pass

#     def get_accel(self):
#         n =  list(motion_mpu6050.accel())
#         n[0] = -n[0]
#         n[2] = -n[2]
#         return tuple([i/65536 for i in n])

#     def get_gyro(self):
#         return tuple([i/65536 for i in motion_mpu6050.gyro()])

#     # output following: Pitch: -180 to 180 Roll: -90 to 90 Yaw: -180 to 180
#     def get_euler(self):
#         n =  list([i/65536 for i in motion_mpu6050.euler()])
#         n[0] = (-(n[0]+180) if n[0]<0 else -(n[0]-180))
#         n[1] = -n[1]
#         return tuple(n)
        
#     # output w x y z
#     def get_quat(self):
#         return tuple([i/math.pow(2, 30) for i in motion_mpu6050.quat()])

# class PinMode(object):
#     IN = 1
#     OUT = 2
#     PWM = 3
#     ANALOG = 4
#     OUT_DRAIN = 5


# pins_remap_esp32 = (33, 32, 35, 34, 39, 0, 16, 17, 26, 25, 36, 2, -1, 18, 19, 21, 5, -1, -1, 22, 23, -1, -1, 27, 14, 12,
#                     13, 15, 4)


# class MPythonPin():
#     def __init__(self, pin, mode=PinMode.IN, pull=None):
#         if mode not in [PinMode.IN, PinMode.OUT, PinMode.PWM, PinMode.ANALOG, PinMode.OUT_DRAIN]:
#             raise TypeError("mode must be 'IN, OUT, PWM, ANALOG,OUT_DRAIN'")
#         if pin == 4:
#             raise TypeError("P4 is used for light sensor")
#         try:
#             self.id = pins_remap_esp32[pin]
#         except IndexError:
#             raise IndexError("Out of Pin range")
#         if mode == PinMode.IN:
#             if pin in [3]:
#                 raise TypeError('IN not supported on P%d' % pin)
#             self.Pin = Pin(self.id, Pin.IN, pull)
#         if mode == PinMode.OUT:
#             if pin in [2, 3]:
#                 raise TypeError('OUT not supported on P%d' % pin)
#             self.Pin = Pin(self.id, Pin.OUT, pull)
#         if mode == PinMode.OUT_DRAIN:
#             if pin in [2, 3]:
#                 raise TypeError('OUT_DRAIN not supported on P%d' % pin)
#             self.Pin = Pin(self.id, Pin.OPEN_DRAIN, pull)
#         if mode == PinMode.PWM:
#             if pin not in [0, 1, 5, 6, 7, 8, 9, 11, 13, 14, 15, 16, 19, 20, 23, 24, 25, 26, 27, 28]:
#                 raise TypeError('PWM not supported on P%d' % pin)
#             self.pwm = PWM(Pin(self.id), duty=0)
#         if mode == PinMode.ANALOG:
#             if pin not in [0, 1, 2, 3, 4, 10]:
#                 raise TypeError('ANALOG not supported on P%d' % pin)
#             self.adc = ADC(Pin(self.id))
#             self.adc.atten(ADC.ATTN_11DB)
#         self.mode = mode

#     def irq(self, handler=None, trigger=Pin.IRQ_RISING):
#         if not self.mode == PinMode.IN:
#             raise TypeError('the pin is not in IN mode')
#         return self.Pin.irq(handler, trigger)

#     def read_digital(self):
#         if not self.mode == PinMode.IN:
#             raise TypeError('the pin is not in IN mode')
#         return self.Pin.value()

#     def write_digital(self, value):
#         if self.mode not in [PinMode.OUT, PinMode.OUT_DRAIN]:
#             raise TypeError('the pin is not in OUT or OUT_DRAIN mode')
#         self.Pin.value(value)

#     def read_analog(self):
#         if not self.mode == PinMode.ANALOG:
#             raise TypeError('the pin is not in ANALOG mode')
#         # calibration esp32 ADC 
#         calibration_val = 0
#         val = int(sum([self.adc.read() for i in range(50)]) / 50)
#         if 0 < val <= 2855:
#             calibration_val = 1.023 * val + 183.6
#         if 2855 < val <= 3720:
#             calibration_val = 0.9769 * val + 181
#         if 3720 < val <= 4095:
#             calibration_val = 4095 - (4095 - val) * 0.2
#         return calibration_val

#     def write_analog(self, duty, freq=1000):
#         if not self.mode == PinMode.PWM:
#             raise TypeError('the pin is not in PWM mode')
#         self.pwm.freq(freq)
#         self.pwm.duty(duty)

# class wifi:
#     def __init__(self):
#         self.sta = network.WLAN(network.STA_IF)
#         self.ap = network.WLAN(network.AP_IF)

#     def connectWiFi(self, ssid, passwd, timeout=10):
#         if self.sta.isconnected():
#             self.sta.disconnect()
#         self.sta.active(True)
#         list = self.sta.scan()
#         for i, wifi_info in enumerate(list):
#             try:
#                 if wifi_info[0].decode() == ssid:
#                     self.sta.connect(ssid, passwd)
#                     wifi_dbm = wifi_info[3]
#                     break
#             except UnicodeError:
#                 self.sta.connect(ssid, passwd)
#                 wifi_dbm = '?'
#                 break
#             if i == len(list) - 1:
#                 raise OSError("SSID invalid / failed to scan this wifi")
#         start = time.time()
#         print("Connection WiFi", end="")
#         while (self.sta.ifconfig()[0] == '0.0.0.0'):
#             if time.ticks_diff(time.time(), start) > timeout:
#                 print("")
#                 raise OSError("Timeout!,check your wifi password and keep your network unblocked")
#             print(".", end="")
#             time.sleep_ms(500)
#         print("")
#         print('WiFi(%s,%sdBm) Connection Successful, Config:%s' % (ssid, str(wifi_dbm), str(self.sta.ifconfig())))

#     def disconnectWiFi(self):
#         if self.sta.isconnected():
#             self.sta.disconnect()
#         self.sta.active(False)
#         print('disconnect WiFi...')

#     def enable_APWiFi(self, essid, password=b'',channel=10):
#         self.ap.active(True)
#         if password:
#             authmode=4
#         else:
#             authmode=0
#         self.ap.config(essid=essid,password=password,authmode=authmode, channel=channel)

#     def disable_APWiFi(self):
#         self.ap.active(False)
#         print('disable AP WiFi...')



# # display
# # if 61 in i2c.scan():
# #     oled = OLED()
# #     # display = oled

# # 3 axis accelerometer
# # accelerometer = Accelerometer()
# # motion = Motion()

# # bm280
# # if 119 in i2c.scan():
# #     bme280 = BME280()

# # 3 rgb leds
# rgb = NeoPixel(Pin(13, Pin.OUT), 6, 3, 1)
# rgb.write()

# # light sensor
# light = ADC(Pin(39))
# light.atten(light.ATTN_11DB)

# # # sound sensor
# # sound = ADC(Pin(36))
# # sound.atten(sound.ATTN_11DB)


# # buttons
# button_b = Pin(2, Pin.IN, Pin.PULL_UP)

# from gui import *


# def numberMap(inputNum, bMin, bMax, cMin, cMax):
#     outputNum = 0
#     outputNum = ((cMax - cMin) / (bMax - bMin)) * (inputNum - bMin) + cMin
#     return outputNum
