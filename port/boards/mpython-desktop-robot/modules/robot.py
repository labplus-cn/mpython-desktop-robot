from mpython import i2c, rgb, button_b
from machine import Pin
import time, ustruct
import esp32
from hcsr04 import HCSR04

class Tcs34725(object):
    """ 

    | 颜色传感器类

    | 颜色传感器可用于两种类型场景：

        - 反射型探测
            须打开补光灯，此场景可识别物体颜色。由于补光灯非纯白，
            此模式下须做亮暗平衡，且须调用balance_enable()函数使能亮暗平衡功能。
        - 探测光源
            此应用场景须关闭颜色补光灯，此状态不需做白平衡，可用于探测光源颜色、色温、亮度。

    | 使用颜色传感器必须先调用sensor_init()初始化。
    """

    def __init__(self):
        pass

    def set_led(self, on):
        """ 
        打开/关闭补光灯

        :param on: bool True 打开 False 关闭
        """
        if on:
            i2c.writeto(17, b'\x8A') # color led on
        else:
            i2c.writeto(17, b'\x8B') # color led off

    def init(self):
        """ 
        颜色传感器初始化，使用颜色传感前，必须先调用本函数初始化。
        """
        # self.set_led(True)
        i2c.writeto(17, b'\x8C') # color sensor init

    def deinit(self):
        """ 
        颜色传感器去初始化，不需要使用颜色传感器时，可调用本函数。
        """
        # self.set_led(False)
        i2c.writeto(17, b'\x8D') # color sensor deinit

    def get_raw(self):
        """ 
        获取裸数据

        :return: tuple 裸数据元组(r,g,b,c)
        """
        i2c.writeto(17, b'\x03', False)        
        raw = ustruct.unpack('HHHH',i2c.readfrom(17, 8))
        return raw

    def get_rgb(self):
        """ 
        获取RGB值，通过反射方式获取物体颜色时，须开启亮暗平衡调整功能。

        :return: tuple RGB颜色值元组(r,g,b)
        """
        i2c.writeto(17, b'\x04', False)       
        rgb = ustruct.unpack('BBB',i2c.readfrom(17, 3))
        return rgb     

    def get_hsv(self):
        """ 
        获取hsv值，通过反射方式获取物体颜色时，须开启亮暗平衡调整功能。

        :return: tuple hsv颜色值元组(h,s,v)
        """
        i2c.writeto(17, b'\x05', False)       
        hsv = ustruct.unpack('Iff',i2c.readfrom(17, 12))
        return hsv

    def get_temp(self):
        """ 
        获取色温值

        :return: int 色温值
        """
        self.set_led(False)
        i2c.writeto(17, b'\x06', False)   
        color_temp = ustruct.unpack('H',i2c.readfrom(17, 2))[0]
        return color_temp

    def get_lux(self):
        """ 
        获取亮度值

        :return: int 亮度值，单位lux
        """
        self.set_led(False)
        i2c.writeto(17, b'\x08', False)   
        lux = ustruct.unpack('H',i2c.readfrom(17, 2))[0]
        return lux

    def set_balance(self, mode):
        """ 
        设置亮暗平衡调整参数，设置后的参数值，会保存到flash中，所以通常本函数只需执行一次。
        
        :param mode: 1:设置亮平衡 2：设置暗平衡
        """
        if mode == 1:
            i2c.writeto(17, b'\xF2')  # white balance
        elif mode == 2:
             i2c.writeto(17, b'\xF3')   # black balance   

    def balance_en(self, en):
        """ 
        使能/禁能亮暗平衡调整功能，获取反射型物体颜色时，须调用本函数使能亮暗平衡调整功能。
        
        :param en: True:使能 False：禁能
        """
        if en:
            i2c.writeto(17, b'\x97')
        else:
            i2c.writeto(17, b'\x98')

class Tracking(object):
    """
    | 机器人控制类。
    | 本类实现机器人相关功能：颜色识别、防跌落、循迹、机器人运动及省电功能控制。
    """
    def __init__(self):
        self.tracking_led_on = False

    def set_led(self, on):
        """ 
        循迹传感器led开关，使用循迹传感器前需打开LED。
        
        :param on: True 开 False 关
        """
        if on:
            i2c.writeto(17, b'\x86') 
        else:
            i2c.writeto(17, b'\x87') 

    def pid_tracking(self, enable):
        """ 
        使能/禁能内置PID循迹功能
        
        :param enable: True 使能 False 禁止
        """
        if enable:
            i2c.writeto(17, b'\x88')
        else:
            i2c.writeto(17, b'\x89') 

    def detect_param(self):
        """ 
        获取循迹图上的黑白AD采样值，改善循迹效果。执行本函数后，相关参数存在flash，因此通常只需执行一次。
        """
        i2c.writeto(17, b'\xF4') 

    def get_val(self):
        """ 
        获取循迹AD采样值

        :return: 四个循迹传感器AD采样值
        """
        if not self.tracking_led_on :
            self.tracking_led_on = True
            self.set_led(1)
        i2c.writeto(17, b'\x01', False) # 获取AD值
        tmp = ustruct.unpack('HHHHHHHH',i2c.readfrom(17, 16))
        tracking_data = tmp[0:4]
        return tracking_data       

class DropDetect(object):
    def __init__(self):
        self.drop_detct_ir_on = False # 防跌落红外发射管状态，耗电较大，不用时须关闭

    def set_ir(self, on):
        """ 
        防跌落传感器发射管开关，使用防跌落传感器需打开发射管。
        
        :param on: True 打开 False 关闭
        """
        if on:
            i2c.writeto(17, b'\x8E') 
        else:
            i2c.writeto(17, b'\x8F') 
    
    def auto_detect(self, enable):
        """ 
        开启/禁止内置防跌落功能, 开启本功能后，探测到机器人悬空后，机器人停止行走。
        
        :param enable: True 开启 False 关闭
        """
        if enable:
            i2c.writeto(17, b'\x8E') #打开防跌红外发射管
            i2c.writeto(17, b'\x90')
        else:
            i2c.writeto(17, b'\x8F') #关闭防跌红外发射管
            i2c.writeto(17, b'\x91')
    
    def get_sensor_val(self):
        """ 
        获取防跌落传感器AD采样值

        :return: tuple 四个防跌落传感器AD采样值
        """
        if self.drop_detct_ir_on == False:
            self.drop_detct_ir_on = True
            i2c.writeto(17, b'\x8E') #打开防跌红外发射管
        i2c.writeto(17, b'\x01', False) #发出读ADC数据指令   
        tmp = ustruct.unpack('HHHHHHHH',i2c.readfrom(17, 16))
        drop_detect_data = tmp[4:8]     
        return drop_detect_data 

    def set_threshold(self, threshold):
        """ 
        设置跌落检测阈值

        :param threshold: 四个角的防跌落检测阈值。 
        """
        tmp = ustruct.pack('<b4H', 0x96, threshold[0], threshold[1], threshold[2], threshold[3])
        i2c.writeto(17, tmp) 

class RobotMotion(object):
    def set_motor(self, num, speed):
        """ 
        控制单个马达
        
        :param num: 马达号， 1：左马达 2：右马达
        :param speed: 速度 取值范围： -100 -- 100
        """
        tmp = ustruct.pack('<3b', 0x94, num, speed)
        i2c.writeto(17, tmp) 

    def move(self, speed, ditance = 0xffffffff):
        """ 
        机器人前进、后退

        :param speed: 速度 取值范围： -100 -- 100 正值代表前进
        :param distance: 前时后退时，表示移动距离 0 -- 0xffffffff，单位:mm 为0xffffffff时表示持续行走。 
        """
        tmp = ustruct.pack('<b2I', 0x92, speed, ditance)
        i2c.writeto(17, tmp) 

    def rotate(self, speed, angle):
        """ 
        机器人旋转

        :param speed: 速度 取值范围： 0 -- 100
        :param angel: 转角，单位:度 取值范围：-360 -- 360  正值代表右转
        """
        tmp = ustruct.pack('<b2H', 0x93, speed, angle)
        i2c.writeto(17, tmp) 

    def compensation(self, mode, val):
        """ 
        机器人行走参数补偿

            制造上的误差（如齿轮箱，轮子直径、轴距等），会影响行走的精确性。
            本函数提供补偿功能。
        
        :param mode: 1: 移动补偿，步进一步补偿值 2：旋转补偿，旋转一度的补偿值。
        :param val: 补偿值，浮点数，可为正负值。
        """
        if mode == 1:
            tmp = ustruct.pack('<bf', 0xF6, val)
            print(tmp)
            i2c.writeto(17, tmp) 
        elif mode == 2:
            tmp = ustruct.pack('<bf', 0xF7, val)
            i2c.writeto(17, tmp) 

class SysConfig(object):
    def get_params(self):
        """ 
        获取机器人所有配置参数
        
        :return: 示例：(85, 0.002, 0.005, 1.0, 1.0, 1.0,  8, 11, 10, 30, 254, 3411, 3157, 259, 3444, 3185, 254, 3498, 3244, 252, 3305, 3053, 2000, 2000, 2000, 2000)  
        | 85: 标记   
        | 0.002: 行走补偿 
        | 0.005: 旋转补偿  
        | 1.0, 1.0, 1.0: 亮平衡RGB比例值  
        | 8, 11, 10, 30: 暗平衡rgbc值  
        | 254, 3411, 3157, 259: 循迹传感器1白平衡亮、值、差值 有4组  
        """
        i2c.writeto(17, b'\x71', False)   
        sys_params = ustruct.unpack('I5f20H',i2c.readfrom(17, 56))
        return sys_params

    def set_default_params(self):
        """ 
        设置缺省的参数。配置参数调乱时，可调用本函数恢复缺省参数。
        """
        i2c.writeto(17, b'\xf1') 

    def _power_save(self, mode = 3):
        """ 
        stm32及外设进入省电模式，默认进省电模式3(待机模式)

        :param mode: 1: sleep模式 2：stop模式 3：standby模式
        """
        if mode ==1:
            i2c.writeto(17, b'\x81')
        elif mode == 2:
            i2c.writeto(17, b'\x82')
        elif mode == 3:
            i2c.writeto(17, b'\x83')

class Hand():
    left = 1
    right =2

class RobotHandColor(object):
    """机器人左右手颜色设置类"""

    def __init__(self):
        # for i in range(0,8):
        #     rgb[i] = (0,0,0)
        # rgb.write()
        pass

    def set_color(self,hand, color):
        """ 
        设置机器人左右手颜色。

        :param hand: Hand.left: 左手 Hand.right：右手
        :param color: 颜色值(r,g,b)
        """
        if hand == 1: # left hand
            for i in range(0,4):
                rgb[i] = color
            rgb.write()            
        elif hand == 2: # right hand
            for i in range(4,8):
                rgb[i] = color
            rgb.write()

    def clear(self, hand):
        """ 
        清除机器人左右手颜色。

        :param hand: Hand.left: 左手 Hand.right：右手
        """
        if hand == 1: # left hand
            for i in range(0,4):
                rgb[i] = (0,0,0)
            rgb.write()            
        elif hand == 2: # right hand
            for i in range(4,8):
                rgb[i] = (0,0,0)
            rgb.write()  

class Ultrasonic(object):
    """超声波测距传感器类"""

    def __init__(self):
        self.hcsr04 = HCSR04(trigger_pin=Pin.P14, echo_pin=Pin.P15)

    def distance_mm(self):
        """ 
        获取超声波测距值。

        :return: 返回超声波测距值，单位：mm
        """
        return self.hcsr04.distance_mm()

class Button(object):
    """顶部按键类"""


    def __init__(self, callback):
        self.cb1 = callback
        button_b.irq(trigger=Pin.IRQ_FALLING, handler=self._on_button_pressed)

    def _on_button_pressed(self, _):
        """ 
        按键按下回调函数，本函数会调用用户回调函数，用户可在自定义的回
        调函数中编写按键响应代码。本函数不需要被用户调用。
        """
        time.sleep_ms(50)
        if button_b.value() == 1: 
            return
        self.cb1()

color_sensor = Tcs34725()
"""颜色传感器实例"""
tracking_sensor = Tracking()
"""循迹传感器实例"""
drop_sensor = DropDetect()
"""防跌落传感器实例"""
robot_motion = RobotMotion()
"""机器人运动传感器实例"""
sys_cfg = SysConfig()
"""机器人系统配置类实例"""
ultrasonic = Ultrasonic()
"""超声波传感器实例"""
hand_color = RobotHandColor()
"""机器人左右手颜色显示类实例"""
