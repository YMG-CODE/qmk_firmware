#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "quantum.h"
#include "pointing_device.h"

report_mouse_t ete_pointing_tune(report_mouse_t report);
void ete_toggle_lr(void);
bool ete_get_swap_state(void);

uint8_t ETE_get_cpi(void);
void    ETE_set_cpi(uint8_t cpi);

bool    ETE_get_scroll_mode(void);
void    ETE_set_scroll_mode(bool mode);

// scroll mode (hold)
void ete_set_scroll_hold(bool on);
bool ete_get_scroll_hold(void);

// scroll speed
void ete_scroll_speed_inc(void);
void ete_scroll_speed_dec(void);
uint8_t ete_get_scroll_speed(void);

// save/load (explicit save)
void ete_cursor_speed_inc(void);
void ete_cursor_speed_dec(void);
uint8_t ete_get_cursor_speed(void);

void ete_settings_save(void);

// inertia
uint8_t ete_get_inertia(void);
void ete_inertia_inc(void);
void ete_inertia_dec(void);
void ete_toggle_inertia(void);

// ===== Feature Switches =====
// rules.mk で -DETE_ENABLE_xxx を指定する

// #define ETE_ENABLE_I2C
// #define ETE_ENABLE_RAW
// #define ETE_ENABLE_SOLENOID

#define ETE_CORE2_I2C_ADDR 0x20  // Core2 側と一致させる

// ★ これを必ず追加
void ete_solenoid_light(void);


// ===== 初期化 =====
void ete_init(void);

// ===== キーイベント =====
void ete_on_key(uint16_t keycode, keyrecord_t *record);

// ===== レイヤー変更 =====
void ete_on_layer(uint8_t layer);

// ===== 定期処理 (matrix_scan_user から呼ぶ) =====
void ete_tick(void);







