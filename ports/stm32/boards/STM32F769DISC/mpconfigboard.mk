MCU_SERIES = f7
CMSIS_MCU = STM32F769xx
MICROPY_FLOAT_IMPL = double
AF_FILE = boards/stm32f767_af.csv
LD_FILES = boards/stm32f769.ld boards/common_ifs.ld
TEXT0_ADDR = 0x08000000
TEXT1_ADDR = 0x08020000