# path to ADF and it's components
# ifeq ($(ADF_PATH),)
# $(info The ADF_PATH variable has not been set, please set it to the root of the esp-adf repository.)
# $(error ADF_PATH not set)
# endif

ESPADF = lib/esp-adf
ADFCOMP = lib/esp-adf/components
ESPCOMP_KCONFIGS += $(shell find $(ADFCOMP) -name Kconfig)
ESPCOMP_KCONFIGS_PROJBUILD += $(shell find $(ADFCOMP) -name Kconfig.projbuild)

ADF_VER = := $(shell git -C $(ESPADF) describe)

CFLAGS_COMMON += -Wno-sign-compare

# bluetooth_service clouds dueros_service esp_actions input_key_service playlist wifi_service
INC += -I$(BOARD_DIR)/esp-adf/mod/include
INC += -I$(BOARD_DIR)/esp-adf/mod/dueros/main

INC_ESPCOMP += -I$(BOARD_DIR)/esp-adf/board/include

# INC_ESPCOMP += -I$(ADFCOMP)/audio_hal/include
# INC_ESPCOMP += -I$(ADFCOMP)/audio_hal/driver/include
# INC_ESPCOMP += -I$(ADFCOMP)/audio_hal/driver/es8388
INC_ESPCOMP += -I$(ADFCOMP)/audio_pipeline/include
INC_ESPCOMP += -I$(ADFCOMP)/audio_sal/include
INC_ESPCOMP += -I$(ADFCOMP)/audio_stream/include
INC_ESPCOMP += -I$(ADFCOMP)/audio_stream
INC_ESPCOMP += -I$(ADFCOMP)/display_service/include
INC_ESPCOMP += -I$(ADFCOMP)/display_service/led_indicator/include
INC_ESPCOMP += -I$(ADFCOMP)/display_service/led_bar/include
INC_ESPCOMP += -I$(ADFCOMP)/esp_dispatcher/include
INC_ESPCOMP += -I$(ADFCOMP)/esp_peripherals/include
INC_ESPCOMP += -I$(ADFCOMP)/esp_peripherals/lib/blufi
INC_ESPCOMP += -I$(ADFCOMP)/esp_peripherals/lib/button
INC_ESPCOMP += -I$(ADFCOMP)/esp_peripherals/lib/gpio_isr
INC_ESPCOMP += -I$(ADFCOMP)/esp_peripherals/lib/sdcard
INC_ESPCOMP += -I$(ADFCOMP)/esp-adf-libs/esp_audio/include
INC_ESPCOMP += -I$(ADFCOMP)/esp-adf-libs/esp_codec/include/codec
INC_ESPCOMP += -I$(ADFCOMP)/esp-adf-libs/esp_codec/include/processing
INC_ESPCOMP += -I$(ADFCOMP)/esp-adf-libs/recorder_engine/include
INC_ESPCOMP += -I$(ADFCOMP)/esp-adf-libs/esp_ssdp/include
INC_ESPCOMP += -I$(ADFCOMP)/esp-adf-libs/esp_dlna/include
INC_ESPCOMP += -I$(ADFCOMP)/esp-adf-libs/esp_upnp/include
INC_ESPCOMP += -I$(ADFCOMP)/esp-adf-libs/esp_sip/include
INC_ESPCOMP += -I$(ADFCOMP)/esp-adf-libs/audio_misc/include
INC_ESPCOMP += -I$(ADFCOMP)/playlist/include
INC_ESPCOMP += -I$(ADFCOMP)/adf_utils/include
INC_ESPCOMP += -I$(ADFCOMP)/adf_utils/cloud_services/include
INC_ESPCOMP += -I$(ADFCOMP)/clouds/dueros/lightduer/include
INC_ESPCOMP += -I$(ADFCOMP)/dueros_service/include

# INC_ESPCOMP += -I$(ESPCOMP)/esp_http_client/include
# INC_ESPCOMP += -I$(ESPCOMP)/esp_http_client/lib/include
# INC_ESPCOMP += -I$(ESPCOMP)/spiffs/include
# INC_ESPCOMP += -I$(ESPCOMP)/spiffs/src
# INC_ESPCOMP += -I$(ESPCOMP)/fatfs/src
# INC_ESPCOMP += -I$(ESPCOMP)/fatfs/diskio
# INC_ESPCOMP += -I$(ESPCOMP)/fatfs/vfs
INC_ESPCOMP += -I$(ESPCOMP)/esp_adc_cal/include
# INC_ESPCOMP += -I$(ESPCOMP)/wear_levelling/include
# INC_ESPCOMP += -I$(ESPCOMP)/wear_levelling/private_include
# INC_ESPCOMP += -I$(ESPCOMP)/tcp_transport/include
# INC_ESPCOMP += -I$(ESPCOMP)/esp-tls
