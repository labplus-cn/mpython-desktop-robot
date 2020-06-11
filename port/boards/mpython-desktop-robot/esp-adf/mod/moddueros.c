/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2019 <ESPRESSIF SYSTEMS (SHANGHAI) CO., LTD>
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

#include <stdio.h>
#include <string.h>
#include "py/objstr.h"
#include "py/runtime.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "dueros_app.h"
#include "es8388.h"
#include "esp_audio.h"

extern esp_audio_handle_t player;

STATIC mp_obj_t dueros_init(mp_obj_t _i2c)
{
    if(!es_i2c_obj){
        es_i2c_obj = (mp_obj_base_t *)_i2c;
    }
    duer_app_init();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(dueros_init_obj, dueros_init);

STATIC mp_obj_t audio_player_get_vol(void)
{
    int vol = 0;
    if (!player)
        mp_raise_ValueError(MP_ERROR_TEXT("No player!"));
    esp_audio_vol_get(player, &vol);
    return mp_obj_new_int(vol);

}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(audio_player_get_vol_obj, audio_player_get_vol);

STATIC mp_obj_t audio_player_set_vol(mp_obj_t vol)
{
    int volume = mp_obj_get_int(vol);
    if (!player)
        mp_raise_ValueError(MP_ERROR_TEXT("No player!"));
    return mp_obj_new_int(esp_audio_vol_set(player, volume));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(audio_player_set_vol_obj, audio_player_set_vol);

STATIC const mp_rom_map_elem_t dueros_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_audio) },
    { MP_ROM_QSTR(MP_QSTR_dueros_init), MP_ROM_PTR(&dueros_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_vol), MP_ROM_PTR(&audio_player_get_vol_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_vol), MP_ROM_PTR(&audio_player_set_vol_obj) },
};

STATIC MP_DEFINE_CONST_DICT(dueros_module_globals, dueros_module_globals_table);

const mp_obj_module_t dueros_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&dueros_module_globals,
};
