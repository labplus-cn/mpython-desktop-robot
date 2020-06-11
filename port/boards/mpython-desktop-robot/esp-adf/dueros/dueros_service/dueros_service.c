/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_audio_device_info.h"
#include "lightduer_ota_notifier.h"
#include "lightduer_voice.h"
#include "lightduer_connagent.h"
#include "lightduer_dcs.h"

#include "extmod/vfs_fat.h"
#include "py/builtin.h"
#include "py/runtime.h"
#include "py/stream.h"

#include "board.h"
#include "sdkconfig.h"
#include "audio_mem.h"
#include "dueros_service.h"
#include "recorder_engine.h"
#include "esp_audio.h"
#include "esp_log.h"
#include "esp_err.h"

#define DUEROS_TASK_PRIORITY        5
#define DUEROS_TASK_STACK_SIZE      6*1024
#define RECORD_SAMPLE_RATE          (16000)

#define RECORD_DEBUG                0

static const char *TAG              = "DUEROS";
static bool                         duer_login_success;

typedef enum {
    DUER_CMD_UNKNOWN,
    DUER_CMD_LOGIN,
    DUER_CMD_CONNECTED,
    DUER_CMD_START,
    DUER_CMD_STOP,
    DUER_CMD_QUIT,
    DUER_CMD_DESTROY,
} duer_task_cmd_t;

typedef struct {
    xQueueHandle            duer_que;
    service_state_t         duer_state;
} dueros_service_t;

typedef struct {
    duer_task_cmd_t     type;
    uint32_t            *pdata;
    int                 index;
    int                 len;
} duer_task_msg_t;

static audio_service_handle_t duer_serv_handle = NULL;

static char *duer_profile_read(const char *path, int16_t *length)
{
    mp_obj_t file;
    mp_obj_t tell;
    mp_obj_t args[2];

    args[0] = mp_obj_new_str(path, strlen(path));
    args[1] = mp_obj_new_str("rb", strlen("rb"));
    file = mp_vfs_open(2, args, (mp_map_t *)&mp_const_empty_map);
    if (file == mp_const_none) {
        ESP_LOGE(TAG, "Failed to open file %s", path);
        return NULL;
    }
    
    mp_load_method(file, MP_QSTR_tell, &tell);
    mp_stream_posix_lseek(file, 0, SEEK_END);
    mp_obj_t len = mp_call_function_1(tell, file);
    mp_stream_posix_lseek(file, 0, SEEK_SET);
    int16_t sz = (int)mp_obj_get_int(len);
    *length = sz;
    ESP_LOGI(TAG, "File size is %d byte", sz);

    char *buffer = audio_calloc_inner(1, sz);
    int rlen = mp_stream_posix_read(file, buffer, sz);
    ESP_LOGI(TAG, "File: %s", buffer);
    if (rlen <= 0) {
        ESP_LOGW(TAG, "No more data,ret:%d", rlen);
    }

    mp_stream_close(file);
    return buffer;
}

static void duer_que_send(void *que, duer_task_cmd_t type, void *data, int index, int len, int dir)
{
    duer_task_msg_t evt = {0};
    evt.type = type;
    evt.pdata = data;
    evt.index = index;
    evt.len = len;
    if (dir) {
        xQueueSendToFront(que, &evt, 0) ;
    } else {
        xQueueSend(que, &evt, 0);
    }
}

static void report_info_task(void *pvParameters)
{
    int ret;
    ret = duer_report_device_info();
    if (ret != DUER_OK) {
        ESP_LOGE(TAG, "Report device info failed ret:%d", ret);
    }
    vTaskDelete(NULL);
}
static void duer_dcs_init(void)
{
    static bool is_first_time = true;
    ESP_LOGI(TAG, "duer_dcs_init");
    duer_dcs_framework_init();
    duer_dcs_voice_input_init();
    duer_dcs_voice_output_init();
    duer_dcs_speaker_control_init();
    duer_dcs_audio_player_init();

    if (is_first_time) {
        is_first_time = false;
        duer_dcs_sync_state();
    }
}

static void duer_event_hook(duer_event_t *event)
{
    if (!event) {
        ESP_LOGE(TAG, "NULL event!!!");
        return;
    }
    ESP_LOGI(TAG, "event: %d", event->_event);
    switch (event->_event) {
        case DUER_EVENT_STARTED:
            // Initialize the DCS API
            duer_dcs_init();
            dueros_service_t *serv = audio_service_get_data(duer_serv_handle);
            duer_que_send(serv->duer_que, DUER_CMD_CONNECTED, NULL, 0, 0, 0);
            duer_login_success = true;
            ESP_LOGI(TAG, "event: DUER_EVENT_STARTED");
            xTaskCreate(&report_info_task, "report_info_task", 1024 * 2, NULL, 5, NULL);
            break;
        case DUER_EVENT_STOPPED:
            ESP_LOGI(TAG, "event: DUER_EVENT_STOPPED");
            duer_login_success = false;
            audio_service_disconnect(duer_serv_handle);
            break;
    }
}

static void duer_login(void)
{
    int16_t len;
    char *buf = duer_profile_read("duer_profile", &len);

    // char *data = audio_calloc_inner(1, bufinfo.len);
    if (NULL == buf) {
        ESP_LOGE(TAG, "audio_malloc failed");
        return;
    }
    // memcpy(data, bufinfo.buf, bufinfo.len);
    // ESP_LOGI(TAG, "duer_start, len:%d\n%s", bufinfo.len, data);
    duer_start(buf, len);
    audio_free((void *)buf);
}

static void dueros_task(void *pvParameters)
{
    /** We get a delay here because of a known issue, or there will be a crash*/
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    audio_service_handle_t serv_handle = (audio_service_handle_t)pvParameters;
    dueros_service_t *serv = audio_service_get_data(serv_handle);
    duer_initialize();
    duer_set_event_callback(duer_event_hook);
    duer_init_device_info();

    uint8_t *voiceData = audio_calloc(1, REC_ONE_BLOCK_SIZE);
    if (NULL == voiceData) {
        ESP_LOGE(TAG, "Func:%s, Line:%d, Malloc failed", __func__, __LINE__);
        goto dueros_task_fail;
    }
    static duer_task_msg_t duer_msg;
    FILE *file = NULL;
#if RECORD_DEBUG
    file = fopen("/sdcard/rec_adf_1.wav", "w+");
    if (NULL == file) {
        ESP_LOGW(TAG, "open rec_adf_1.wav failed,[%d]", __LINE__);
    }
#endif
    int task_run = 1;
    serv->duer_state = SERVICE_STATE_IDLE;
    service_event_t serv_evt = {
        .type = 0,
        .source = serv,
        .data = &serv->duer_state,
        .len = 0,
    };
    while (task_run) {
        if (xQueueReceive(serv->duer_que, &duer_msg, portMAX_DELAY)) {
            if (duer_msg.type == DUER_CMD_LOGIN) {
                ESP_LOGE(TAG, "Recv Que DUER_CMD_LOGIN");
                if (serv->duer_state == SERVICE_STATE_IDLE) {
                    duer_login();
                    serv->duer_state = SERVICE_STATE_CONNECTING;
                    audio_service_callback(serv_handle, &serv_evt);
                } else {
                    ESP_LOGW(TAG, "DUER_CMD_LOGIN connecting,duer_state = %d", serv->duer_state);
                }
            } else if (duer_msg.type == DUER_CMD_CONNECTED) {
                ESP_LOGI(TAG, "Dueros DUER_CMD_CONNECTED, duer_state:%d", serv->duer_state);
                serv->duer_state = SERVICE_STATE_CONNECTED;
                audio_service_callback(serv_handle, &serv_evt);
            } else if (duer_msg.type == DUER_CMD_START) {
                if (serv->duer_state < SERVICE_STATE_CONNECTED) {
                    ESP_LOGW(TAG, "Dueros has not connected, state:%d", serv->duer_state);
                    continue;
                }
                ESP_LOGI(TAG, "Recv Que DUER_CMD_START");
                duer_voice_start(RECORD_SAMPLE_RATE);
                duer_dcs_on_listen_started();
                serv->duer_state = SERVICE_STATE_RUNNING;
                audio_service_callback(serv_handle, &serv_evt);
                while (1) {
                    int ret = rec_engine_data_read(voiceData, REC_ONE_BLOCK_SIZE, 110 / portTICK_PERIOD_MS);
                    ESP_LOGD(TAG, "index = %d", ret);
                    if ((ret == 0) || (ret == -1)) {
                        break;
                    }
                    if (file) {
                        fwrite(voiceData, 1, REC_ONE_BLOCK_SIZE, file);
                    }
                    ret  = duer_voice_send(voiceData, REC_ONE_BLOCK_SIZE);
                    if (ret < 0) {
                        ESP_LOGE(TAG, "duer_voice_send failed ret:%d", ret);
                        break;
                    }
                }
            } else if (duer_msg.type == DUER_CMD_STOP)  {
                ESP_LOGI(TAG, "Dueros DUER_CMD_STOP");
                if (file) {
                    fclose(file);
                }
                duer_voice_stop();
                serv->duer_state = SERVICE_STATE_STOPPED;
                audio_service_callback(serv_handle, &serv_evt);
            } else if (duer_msg.type == DUER_CMD_QUIT && (serv->duer_state != SERVICE_STATE_IDLE))  {
                if (duer_login_success) {
                    duer_stop();
                }
                serv->duer_state = SERVICE_STATE_IDLE;
                audio_service_callback(serv_handle, &serv_evt);
            } else if (duer_msg.type == DUER_CMD_DESTROY)  {
                ESP_LOGI(TAG, "Dueros DUER_CMD_DESTROY");
                duer_voice_stop();
                serv->duer_state = SERVICE_STATE_IDLE;
                task_run = 1;
            }
        }
    }

dueros_task_fail:
    if (voiceData) {
        free(voiceData);
    }
    vQueueDelete(serv->duer_que);
    free(serv);
    vTaskDelete(NULL);
}

esp_err_t dueros_start(audio_service_handle_t handle)
{
    dueros_service_t *serv = audio_service_get_data(handle);
    duer_que_send(serv->duer_que, DUER_CMD_START, NULL, 0, 0, 0);
    return ESP_OK;
}

esp_err_t dueros_stop(audio_service_handle_t handle)
{
    dueros_service_t *serv = audio_service_get_data(handle);
    duer_que_send(serv->duer_que, DUER_CMD_STOP, NULL, 0, 0, 0);
    return ESP_OK;
}

esp_err_t dueros_connect(audio_service_handle_t handle)
{
    dueros_service_t *serv = audio_service_get_data(handle);
    duer_que_send(serv->duer_que, DUER_CMD_LOGIN, NULL, 0, 0, 0);
    return ESP_OK;
}

esp_err_t dueros_disconnect(audio_service_handle_t handle)
{
    dueros_service_t *serv = audio_service_get_data(handle);
    duer_que_send(serv->duer_que, DUER_CMD_QUIT, NULL, 0, 0, 0);
    return ESP_OK;
}

esp_err_t dueros_destroy(audio_service_handle_t handle)
{
    dueros_service_t *serv = audio_service_get_data(handle);
    duer_que_send(serv->duer_que, DUER_CMD_DESTROY, NULL, 0, 0, 0);
    return ESP_OK;
}

service_state_t dueros_service_state_get()
{
    dueros_service_t *serv = audio_service_get_data(duer_serv_handle);
    return serv->duer_state;
}

audio_service_handle_t dueros_service_create(void)
{
    dueros_service_t *serv =  audio_calloc(1, sizeof(dueros_service_t));
    serv->duer_que = xQueueCreate(3, sizeof(duer_task_msg_t));
    serv->duer_state = SERVICE_STATE_UNKNOWN;
    audio_service_config_t duer_cfg = {
        .task_stack = DUEROS_TASK_STACK_SIZE,
        .task_prio  = DUEROS_TASK_PRIORITY,
        .task_core  = 1,
        .task_func  = dueros_task,
        .service_start = dueros_start,
        .service_stop = dueros_stop,
        .service_connect = dueros_connect,
        .service_disconnect = dueros_disconnect,
        .service_destroy = dueros_destroy,
        .service_name = "duer_serv",
        .user_data = serv,
    };
    audio_service_handle_t duer = audio_service_create(&duer_cfg);
    duer_serv_handle = duer;
    return duer;
}