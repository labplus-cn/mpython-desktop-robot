from mpython import *
import time, ustruct
import esp32

class TCS34725():
    ''' 
    颜色传感器类
    颜色传感器可用于两种类型场景：
    1、反射型探测：须打开补光灯，此场景可识别物体颜色。由于补光灯非纯白，
       此模式下须做亮暗平衡，且须调用balance_enable()函数使能亮暗平衡功能。
    2、探测光源：此应用场景须关闭颜色补光灯，此状态不需做白平衡，可用于探测
       光源颜色、色温、亮度。
    使用颜色传感器必须先调用sensor_init()初始化。
    '''

    def __init__(self):
        pass

    def set_led(self, on):
        ''' 
        打开/关闭补光灯
        参数：
            on: 1 打开 0 关闭
        '''
        if on:
            i2c.writeto(17, b'\x8A') # color led on
        else:
            i2c.writeto(17, b'\x8B') # color led off

    def sensor_init(self):
        ''' 
        颜色传感器初始化，使用颜色传感前，必须先调用本函数初始化。
        '''
        self.set_led(1)
        i2c.writeto(17, b'\x8C') # color sensor init

    def sensor_deinit(self):
        ''' 
        颜色传感器去初始化，不需要使用颜色传感器时，可调用本函数。
        '''
        self.set_led(0)
        i2c.writeto(17, b'\x8D') # color sensor deinit

    def get_raw(self):
        ''' 
        获取裸数据
        参数：无
        返回值：裸数据元组(r,g,b,c)
        '''
        i2c.writeto(17, b'\x03', False)        
        raw = struct.unpack('HHHH',i2c.readfrom(17, 8))
        return raw

    def get_rgb(self):
        ''' 
        获取RGB值，通过反射方式获取物体颜色时，须开启亮暗平衡调整功能。
        参数：无
        返回值：RGB颜色值元组(r,g,b)
        '''
        i2c.writeto(17, b'\x04', False)       
        rgb = struct.unpack('BBB',i2c.readfrom(17, 3))
        return rgb     

    def get_hsv(self, balance_en):
        ''' 
        获取hsv值，通过反射方式获取物体颜色时，须开启亮暗平衡调整功能。
        参数：无
        返回值：hsv颜色值元组(h,s,v)
        '''
        i2c.writeto(17, b'\x05', False)       
        hsv = struct.unpack('Iff',i2c.readfrom(17, 12))
        return hsv

    def get_color_temperature(self):
        ''' 
        获取色温值
        参数：无
        返回值：色温值
        '''
        self.set_led(0)
        i2c.writeto(17, b'\x06', False)   
        color_temp = struct.unpack('H',i2c.readfrom(17, 2))
        return color_temp

    def get_lux(self):
        ''' 
        获取亮度值
        参数：无
        返回值：亮度值，单位lux
        '''
        self.set_led(0)
        i2c.writeto(17, b'\x08', False)   
        lux = struct.unpack('H',i2c.readfrom(17, 2))
        return lux

    def set_balance(self, mode):
        ''' 
        设置亮暗平衡调整参数，设置后的参数值，会保存到flash中，所以通常本函数只需执行一次。
        参数：
            mode: 1:设置亮平衡 0：设置暗平衡
        返回值：无
        '''
        if mode == 1:
            i2c.writeto(17, b'\xF2')  # white balance
        elif mode == 2:
             i2c.writeto(17, b'\xF3')   # black balance   

    def balance_enable(self, en):
        ''' 
        使能/禁能亮暗平衡调整功能，获取反射型物体颜色时，须调用本函数使能亮暗平衡调整功能。
        参数：
            en: 1:使能 0：禁能
        返回值：无
        '''
        if en:
            i2c.writeto(17, b'\x97')
        else:
            i2c.writeto(17, b'\x98')

class STM32(object):
    '''
    机器人控制类。
        本类实现机器人相关功能：颜色识别、防跌落、循迹、机器人运动及省电功能控制。
    '''
    def __init__(self):
        self.tracking_led_on = False
        self.drop_detct_ir_on = False # 防跌落红外发射管状态，耗电较大，不用时须关闭
        self.color = TCS34725()

    def _power_save(self, mode = 3):
        ''' 
        stm32及外设进入省电模式，默认进省电模式3(待机模式)
        参数：
            mode: 1: sleep模式 2：stop模式 3：standby模式
        '''
        if mode ==1:
            i2c.writeto(17, b'\x81')
        elif mode == 2:
            i2c.writeto(17, b'\x82')
        elif mode == 3:
            i2c.writeto(17, b'\x83')

    def set_tracking_led(self, on):
        ''' 
        循迹传感器led开关，使用循迹传感器前需打开LED。
        参数：
            on: 1 开 0 关
        '''
        if on:
            i2c.writeto(17, b'\x86') 
        else:
            i2c.writeto(17, b'\x87') 

    def pid_tracking(self, enable):
        ''' 
        使能/禁能内置PID循迹功能
        参数：
            enable: True 使能 False 禁止
        '''
        if enable:
            i2c.writeto(17, b'\x88')
        else:
            i2c.writeto(17, b'\x89') 

    def detect_tracking_param(self):
        ''' 
        获取循迹图上的黑白AD采样值，改善循迹效果。执行本函数后，相关参数存在flash，因此通常只需执行一次。
        '''
        i2c.writeto(17, b'\xF4') 

    def get_tracking_val(self):
        ''' 
        获取循迹AD采样值
        返回值：四个循迹传感器AD采样值
        '''
        if not self.tracking_led_on :
            self.tracking_led_on = True
            self.set_tracking_led(1)
        i2c.writeto(17, b'\x01', False) # 获取AD值
        tmp = struct.unpack('HHHHHHHH',i2c.readfrom(17, 16))
        tracking_data = tmp[0:4]
        return tracking_data       

    def drop_detect_ir(self, on):
        ''' 
        防跌落传感器发射管开关，使用防跌落传感器需打开发射管。
        参数：
            on: True 打开 False 关闭
        '''
        if on:
            i2c.writeto(17, b'\x8E') 
        else:
            i2c.writeto(17, b'\x8F') 
    
    def auto_drop_detect(self, enable):
        ''' 
        开启/禁止内置防跌落功能, 开启本功能后，探测到机器人悬空后，机器人停止行走。
        参数：
            enable: True 开启 False 关闭
        '''
        if enable:
            i2c.writeto(17, b'\x8E') #打开防跌红外发射管
            i2c.writeto(17, b'\x90')
        else:
            i2c.writeto(17, b'\x8F') #关闭防跌红外发射管
            i2c.writeto(17, b'\x91')
    
    def get_drop_sensor_val(self):
        ''' 
        获取防跌落传感器AD采样值
        返回值：四个防跌落传感器AD采样值
        '''
        if self.drop_detct_ir_on == False:
            self.drop_detct_ir_on = True
            i2c.writeto(17, b'\x8E') #打开防跌红外发射管
        i2c.writeto(17, b'\x01', False) #发出读ADC数据指令   
        tmp = struct.unpack('HHHHHHHH',i2c.readfrom(17, 16))
        drop_detect_data = tmp[4:8]     
        return drop_detect_data   

    def set_motor(self, num, speed):
        ''' 
        控制单个马达
        参数：
            num: 马达号， 1：左马达 2：右马达
            speed: 速度 取值范围： -100 -- 100
        '''
        tmp = struct.pack('<3b', 0x96, num, speed)
        i2c.writeto(17, tmp) 

    def robot_move(self, speed, ditance):
        ''' 
        机器人前进、后退
        参数：
            speed: 速度 取值范围： -100 -- 100 正值代表前进
            distance: 前时后退时，表示移动距离 0 -- 0xffffffff，单位:mm 为0xffffffff时表示持续动动 
        '''
        tmp = struct.pack('<b2I', 0x92, speed, ditance)
        i2c.writeto(17, tmp) 

    def robot_rotate(self, speed, angle):
        ''' 
        机器人旋转
        参数：
            speed: 速度 取值范围： 0 -- 100
            angel: 转角，单位:度 取值范围：-360 -- 360  正值代表右转
        '''
        tmp = struct.pack('<b2H', 0x93, speed, angle)
        i2c.writeto(17, tmp) 

    def motor_compensation(self, mode, val):
        ''' 
        机器人行走参数补偿
            制造上的误差（如齿轮箱，轮子直径、轴距等），会影响行走的精确性。
            本函数提供补偿功能。
        参数：
            mode: 1: 移动补偿，步进一步补偿值 2：旋转补偿，旋转一度的补偿值。
            val: 补偿值，浮点数，可为正负值。
        '''
        if mode == 1:
            tmp = struct.pack('<bf', 0xF6, val)
            print(tmp)
            i2c.writeto(17, tmp) 
        elif mode == 2:
            tmp = struct.pack('<bf', 0xF7, val)
            i2c.writeto(17, tmp) 

    def get_sys_params(self):
        ''' 
        获取机器人所有配置参数
        返回值：
            示例：
            (85,    0.0,    0.005,  1.0, 1.0, 1.0,  0, 0, 0, 0,  254, 3411, 3157,     
             标记   行走补偿 旋转补偿  亮平衡RGB比例值  暗平衡rgbc值  循迹传感器1白平衡亮、值、差值
            259, 3444, 3185,            254, 3498, 3244,            252, 3305, 3053)
            循迹传感器2白平衡亮、暗、差值   循迹传感器3白平衡亮、暗、差值  循迹传感器4白平衡亮、暗、差值
        '''
        i2c.writeto(17, b'\x71', False)   
        sys_params = struct.unpack('I5f16H',i2c.readfrom(17, 56))
        return sys_params

    def set_default_sys_params(self):
        ''' 
        设置缺省的参数。配置参数调乱时，可调用本函数恢复缺省参数。
        '''
        i2c.writeto(17, b'\xf1')   
        
stm = STM32()

''' TEST '''
# stm.motor_compensation(1, 0.0)
# time.sleep(2)
# stm.motor_compensation(2, 0.005)
# time.sleep(10)
# stm.detect_tracking_param()
    