
################################################################################
# List of object files from the ESP32 ADF components
ESPADF_AUDIO_BOARD_O = $(patsubst %.c,%.o,$(wildcard $(BOARD_DIR)/esp-adf/board/*.c))

ESPADF_AUDIO_PIPELINE_O = $(patsubst %.c,%.o,$(wildcard $(ADFCOMP)/audio_pipeline/*.c))

ESPADF_AUDIO_SAL_O = $(patsubst %.c,%.o,$(wildcard $(ADFCOMP)/audio_sal/*.c))

ESPADF_AUDIO_STREAM_O = $(patsubst %.c,%.o,\
	$(ADFCOMP)/audio_stream/fatfs_stream.c \
	$(ADFCOMP)/audio_stream/http_stream.c \
	$(ADFCOMP)/audio_stream/i2s_stream.c \
	$(ADFCOMP)/audio_stream/raw_stream.c \
	$(ADFCOMP)/audio_stream/spiffs_stream.c \
	$(ADFCOMP)/audio_stream/i2s_stream.c \
	)

ESPADF_DISPLAY_SERVICE_O = $(patsubst %.c,%.o,\
	$(wildcard $(ADFCOMP)/display_service/*.c) \
	$(wildcard $(ADFCOMP)/display_service/led_bar/*.c) \
	$(wildcard $(ADFCOMP)/display_service/led_indicator/*.c) \
	)

ESPADF_ESP_DISPATCHER_O = $(patsubst %.c,%.o,$(wildcard $(ADFCOMP)/esp_dispatcher/*.c))

ESPADF_ESP_PERIPHERALS_O = $(patsubst %.c,%.o,\
	$(wildcard $(ADFCOMP)/esp_peripherals/*.c) \
	$(wildcard $(ADFCOMP)/esp_peripherals/driver/i2c_bus/*.c) \
	$(wildcard $(ADFCOMP)/esp_peripherals/lib/*/*.c) \
	)

ESPADF_LIBS_O = $(patsubst %.c,%.o,$(wildcard $(ADFCOMP)/esp-adf-libs/esp_codec/*.c))

$(eval $(call gen_espidf_lib_rule,audio_board,$(ESPADF_AUDIO_BOARD_O)))
$(eval $(call gen_espidf_lib_rule,audio_pipeline,$(ESPADF_AUDIO_PIPELINE_O)))
$(eval $(call gen_espidf_lib_rule,audio_sal,$(ESPADF_AUDIO_SAL_O)))
$(eval $(call gen_espidf_lib_rule,audio_stream,$(ESPADF_AUDIO_STREAM_O)))
$(eval $(call gen_espidf_lib_rule,display_service,$(ESPADF_DISPLAY_SERVICE_O)))
$(eval $(call gen_espidf_lib_rule,esp_dispatcher,$(ESPADF_ESP_DISPATCHER_O)))
$(eval $(call gen_espidf_lib_rule,esp_peripherals,$(ESPADF_ESP_PERIPHERALS_O)))
$(eval $(call gen_espidf_lib_rule,esp-adf-libs,$(ESPADF_LIBS_O)))

################################################################################
# List of object files from the ESP32 IDF components which are needed by ADF components

# ESPIDF_HTTP_CLIENT_O = $(patsubst %.c,%.o,
# 	$(wildcard $(ESPCOMP)/esp_http_client/*.c) 
# 	$(wildcard $(ESPCOMP)/esp_http_client/lib/*.c) 
# 	)

ESPIDF_SPIFFS_O = $(patsubst %.c,%.o,$(wildcard $(ESPCOMP)/spiffs/spiffs/src/*.c))

# ESPIDF_FATFS_O = $(patsubst %.c,%.o, 
# 	$(wildcard $(ESPCOMP)/fatfs/src/*.c) 
# 	$(wildcard $(ESPCOMP)/fatfs/diskio/*.c) 
# 	$(wildcard $(ESPCOMP)/fatfs/port/freertos/*.c) 
# 	$(wildcard $(ESPCOMP)/fatfs/vfs/*.c) 
# 	)

ESPIDF_ADC_CAL_O = $(patsubst %.c,%.o,$(wildcard $(ESPCOMP)/esp_adc_cal/*.c))

ESPIDF_WEAR_LEVELLING_O = $(patsubst %.cpp,%.o,$(wildcard $(ESPCOMP)/wear_levelling/*.cpp))

# ESPIDF_TCP_TRANSPORT_O = $(patsubst %.c,%.o,$(wildcard $(ESPCOMP)/tcp_transport/*.c))

# ESPIDF_ESP_TLS_O = $(patsubst %.c,%.o,$(wildcard $(ESPCOMP)/esp-tls/*.c))


# $(eval $(call gen_espidf_lib_rule,esp_http_client,$(ESPIDF_HTTP_CLIENT_O)))
$(eval $(call gen_espidf_lib_rule,spiffs,$(ESPIDF_SPIFFS_O)))
# $(eval $(call gen_espidf_lib_rule,fatfs,$(ESPIDF_FATFS_O)))
$(eval $(call gen_espidf_lib_rule,esp_adc_cal,$(ESPIDF_ADC_CAL_O)))
$(eval $(call gen_espidf_lib_rule,wear_levelling,$(ESPIDF_WEAR_LEVELLING_O)))
# $(eval $(call gen_espidf_lib_rule,tcp_transport,$(ESPIDF_TCP_TRANSPORT_O)))
# $(eval $(call gen_espidf_lib_rule,esp_tls,$(ESPIDF_ESP_TLS_O)))
