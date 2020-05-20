# MicroPython SSD1306 OLED driver, I2C and SPI interfaces

from micropython import const
import framebuf
import time

# register definitions
SET_CONTRAST        = const(0x81)
SET_ENTIRE_ON       = const(0xa4)
SET_NORM_INV        = const(0xa6)
SET_DISP            = const(0xae)
SET_MEM_ADDR        = const(0x20)
SET_COL_ADDR        = const(0x21)
SET_PAGE_ADDR       = const(0x22)
SET_DISP_START_LINE = const(0x40)
SET_SEG_REMAP       = const(0xa0)
SET_MUX_RATIO       = const(0xa8)
SET_COM_OUT_DIR     = const(0xc0)
SET_DISP_OFFSET     = const(0xd3)
SET_COM_PIN_CFG     = const(0xda)
SET_DISP_CLK_DIV    = const(0xd5)
SET_PRECHARGE       = const(0xd9)
SET_VCOM_DESEL      = const(0xdb)
SET_CHARGE_PUMP     = const(0x8d)

SET_ENTIRE_ON1      = const(0xaf)
SET_DISP_OFF        = const(0xae)
SET_DISP_ON         = const(0xaf)
SET_CHARGE_PUMP1    = const(0xad)
SET_COL_ADDR_L      = const(0x02)
SET_COL_ADDR_L1     = const(0x00)
SET_COL_ADDR_H      = const(0x10)
SET_PAGE_ADDR1      = const(0xb0)
SET_CONTRACT_CTRL   = const(0x81)
SET_DUTY            = const(0x3f)
SET_VCC_SOURCE      = const(0x8b)
SET_VPP             = const(0x33)
SET_MCU_PROTECT_STATUS = const(0xfd)
SET_COLUMN_MAP      = const(0xa0)
SET_ROW_SCAN_DIR      = const(0xc0)
SET_SCROLL_DEACTIVE = const(0x2e)

# Subclassing FrameBuffer provides support for graphics primitives
# http://docs.micropython.org/en/latest/pyboard/library/framebuf.html
class SSD1309(framebuf.FrameBuffer):
    def __init__(self, width, height, external_vcc):
        self.width = width
        self.height = height
        self.external_vcc = external_vcc
        self.pages = self.height // 8
        self.buffer = bytearray(self.pages * self.width)
        super().__init__(self.buffer, self.width, self.height, framebuf.MONO_VLSB)
        self.init_display()

    def init_display(self):
        time.sleep_ms(10)
        for cmd in (
                SET_DISP_OFF,   #--turn off oled panel
                SET_MCU_PROTECT_STATUS, 0x12, # Unlock OLED driver
                SET_DISP_CLK_DIV, 0xA0,  #--set display clock divide ratio/oscillator frequency
                SET_MUX_RATIO, 0x3f,    #--set multiplex ratio(1 to 64) --1/64 duty
                SET_DISP_OFFSET, 0x00,  #-set display offset Shift Mapping RAM Counter (0x00~0x3F) -not offset
                SET_DISP_START_LINE,  #--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
                SET_COLUMN_MAP | 0x01, #--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
                SET_ROW_SCAN_DIR | 0x08,  #Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
                SET_COM_PIN_CFG, 0x12, #--set com pins hardware configuration
                SET_CONTRACT_CTRL, 0xbf, #--set contrast control register
                SET_PRECHARGE, 0x25, #--set pre-charge period --Pre-Charge as 15 Clocks & Discharge as 1 Clock
                SET_VCOM_DESEL, 0x34, #--set vcomh
                SET_ENTIRE_ON, # Disable Entire Display On (0xa4/0xa5)
                SET_NORM_INV, # Disable Inverse Display On (0xa6/a7)
                # SET_SCROLL_DEACTIVE, #Deactive scroll.
                SET_DISP_ON # turn on oled
            ): 
            self.write_cmd(cmd)
        self.clear()

    def clear(self):
        for i in range(0,8):
            self.write_cmd(0xb0+i) # 设置页地址（0~7）
            self.write_cmd(0x00)   # 设置显示位置—列低地址
            self.write_cmd(0x10)   # 设置显示位置—列高地址 
            for n in range(0, 128):
                self.write_data(b'\x00')

    def poweroff(self):
        self.write_cmd(0X8D) # SET DCDC命令
        self.write_cmd(0X10) # DCDC ON
        self.write_cmd(SET_DISP | 0x00)

    def poweron(self):
        self.write_cmd(0X8D) # SET DCDC命令
        self.write_cmd(0X14) # DCDC ON
        self.write_cmd(SET_DISP) # DISPLAY ON

    def contrast(self, contrast):
        self.write_cmd(SET_CONTRAST)
        self.write_cmd(contrast)

    def invert(self, invert):
        self.write_cmd(SET_NORM_INV | (invert & 1))

    def show(self):
        for i in range(0, 8):
            self.write_cmd(SET_PAGE_ADDR1 + i)
            self.write_cmd(SET_COL_ADDR_L1 + 0)     # offset 2 pixels for 128x64 panel
            self.write_cmd(SET_COL_ADDR_H + 0)
            self.write_data(self.buffer[i*128:(i+1)*128])   # send one page display data


class SSD1309_I2C(SSD1309):
    def __init__(self, width, height, i2c, addr=0x3c, external_vcc=False):
        self.i2c = i2c
        self.addr = addr
        self.temp = bytearray(2)
        self.write_list = [b'\x40', None] # Co=0, D/C#=1
        super().__init__(width, height, external_vcc)

    def write_cmd(self, cmd):
        self.temp[0] = 0x00 # Co=1, D/C#=0
        self.temp[1] = cmd
        self.i2c.writeto(self.addr, self.temp)

    def write_data(self, buf):
        self.write_list[1] = buf
        self.i2c.writevto(self.addr, self.write_list)


class SSD1306_SPI(SSD1309):
    def __init__(self, width, height, spi, dc, res, cs, external_vcc=False):
        self.rate = 10 * 1024 * 1024
        dc.init(dc.OUT, value=0)
        res.init(res.OUT, value=0)
        cs.init(cs.OUT, value=1)
        self.spi = spi
        self.dc = dc
        self.res = res
        self.cs = cs
        import time
        self.res(1)
        time.sleep_ms(1)
        self.res(0)
        time.sleep_ms(10)
        self.res(1)
        super().__init__(width, height, external_vcc)

    def write_cmd(self, cmd):
        self.spi.init(baudrate=self.rate, polarity=0, phase=0)
        self.cs(1)
        self.dc(0)
        self.cs(0)
        self.spi.write(bytearray([cmd]))
        self.cs(1)

    def write_data(self, buf):
        self.spi.init(baudrate=self.rate, polarity=0, phase=0)
        self.cs(1)
        self.dc(1)
        self.cs(0)
        self.spi.write(buf)
        self.cs(1)
