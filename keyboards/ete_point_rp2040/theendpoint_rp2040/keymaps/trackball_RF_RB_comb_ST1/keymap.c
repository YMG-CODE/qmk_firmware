// Copyright 2023 YMGWorks (@YMGWorks)
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H
#include "quantum.h"
#include <stdio.h>

enum ETE_keycodes {
    ETE_SAFE_RANGE = SAFE_RANGE,
    REC_RST, // ETE configuration: reset to default
    REC_SAVE, // ETE configuration: save to EEPROM

    CPI_I100, // CPI +100 CPI
    CPI_D100, // CPI -100 CPI
    CPI_I1K, // CPI +1000 CPI
    CPI_D1K, // CPI -1000 CPI

    // In scroll mode, motion from primary trackball is treated as scroll
    // wheel.
    SCRL_TO, // Toggle scroll mode
    SCRL_MO, // Momentary scroll mode
    SCRL_DVI, // Increment scroll divider
    SCRL_DVD, // Decrement scroll divider
};

#define REC_RST QK_KB_0
#define REC_SAVE QK_KB_1
#define CPI_I100 QK_KB_2
#define CPI_D100 QK_KB_3
#define CPI_I1K QK_KB_4
#define CPI_D1K QK_KB_5
#define SCRL_TO QK_KB_6
#define SCRL_MO QK_KB_7
#define SCRL_DVI QK_KB_8
#define SCRL_DVD QK_KB_9

//#include "i2c_master.h"
#include "timer.h"
#include "print.h"
#include "raw_hid.h"
#include "ete_common.h"

// ==== RAW HID function prototypes ====
void send_layer_usb(uint8_t layer);
void send_keyevent_usb(uint16_t keycode, bool pressed, uint8_t layer);


#define SLAVE_ADDR         0x0B
#define CMD_REG_DISPLAY    0x01  // CPM表示コマンド
#define CMD_REG_LAYER      0x02  // レイヤーインジケーターコマンド


enum custom_keycodes {
    SCROLL = SAFE_RANGE,
};

bool set_scrolling = false;

#define SCROLL_DIVISOR_H 32.0
#define SCROLL_DIVISOR_V 32.0

float scroll_accumulated_h = 0;
float scroll_accumulated_v = 0;




const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [0] = LAYOUT(
   //,---------------------------------------------------------------------.     ,-----------------------------------------------------------------------.
      KC_ESC,  KC_Q,    KC_W,    KC_E,   KC_R,    KC_T,    KC_MUTE, KC_SPC,       KC_ENT,  KC_TAB,  KC_Y,    KC_U,    KC_I,    KC_O,   KC_P,     KC_MINS,
  //|--------+--------+--------+--------+--------+--------+--------+---------|  |--------+--------+--------+--------+--------+--------+--------+---------|
      KC_LSFT, KC_A,    KC_S,    KC_D,   KC_F,    KC_G,    KC_HOME, KC_PGDN,      KC_UP,   KC_END,  KC_H,    KC_J,    KC_K,    KC_L,   KC_SCLN,  KC_COLN,
  //|--------+--------+--------+--------+--------+--------+--------+---------|  |--------+--------+--------+--------+--------+--------+--------+---------|
      KC_LCTL, KC_Z,    KC_X,    KC_C,   KC_V,    KC_B,    KC_END,  KC_END,       KC_LEFT, KC_RGHT, KC_N,    KC_M,   KC_COMM,  KC_DOT, KC_SLSH,  KC_BSLS,
  //|--------+--------+--------+--------+--------+--------+--------+---------|  |--------+--------+--------+--------+--------+--------+--------+---------|
      KC_TAB,  KC_LALT, KC_LGUI, KC_PSCR, XXXXXXX,MO(1),  KC_DEL,  KC_HOME,       KC_END,   KC_DOWN, MO(2) ,XXXXXXX,KC_BSPC,  KC_DEL, KC_RALT,  KC_AT 
  //|--------+--------+--------+--------+--------+--------+--------+---------|  |--------+--------+--------+--------+--------+--------+--------+---------|
                                                                                                                     
  ),
    [1] = LAYOUT(
  //,----------------------------------------------------------------------.     ,-----------------------------------------------------------------------.
      KC_CAPS,KC_CAPS, KC_TRNS, KC_F1,   KC_F2,   KC_F3,   KC_TRNS, KC_TRNS,     KC_TRNS, KC_TRNS,  KC_1,   KC_2,    KC_3,    KC_HOME , KC_UP,   KC_END,
  //|--------+--------+--------+--------+--------+--------+--------+---------|  |--------+--------+--------+--------+--------+--------+--------+---------|
      KC_TRNS,KC_TRNS, KC_TRNS, KC_F4,   KC_F5,   KC_F6,   KC_TRNS, KC_TRNS,     KC_TRNS, KC_TRNS,  KC_4,   KC_5,    KC_6,    KC_LEFT,  KC_DOWN, KC_RGHT,
  //|--------+--------+--------+--------+--------+--------+--------+---------|  |--------+--------+--------+--------+--------+--------+--------+---------|
      KC_TRNS,KC_TRNS, KC_TRNS, KC_F7,   KC_F8,   KC_F9,   KC_TRNS, KC_TRNS,     KC_TRNS, KC_TRNS,  KC_7,   KC_8,    KC_9,    KC_0,  KC_MS_WH_UP,KC_MS_WH_DOWN,
  //|--------+--------+--------+--------+--------+--------+--------+---------|  |--------+--------+--------+--------+--------+--------+--------+---------|
      KC_TRNS,KC_TRNS, KC_TRNS, KC_TRNS, XXXXXXX, KC_TRNS, KC_TRNS, KC_TRNS,     KC_TRNS, KC_TRNS,  MO(3),  XXXXXXX, KC_TRNS, KC_TRNS,  KC_TRNS, KC_TRNS
  //|--------+--------+--------+--------+--------+--------+--------+---------|  |--------+--------+--------+--------+--------+--------+--------+---------'
  ),

    [2] = LAYOUT(
  //,------------------------------------------------------------------------.    ,----------------------------------------------------------------------.
     KC_ESC,  KC_EXLM, KC_AT,   KC_HASH,  KC_DLR,  KC_PERC, KC_TRNS, KC_TRNS,    KC_TRNS, KC_TRNS, KC_CIRC, KC_QUOT, KC_DQUO, KC_LPRN, KC_RPRN, KC_UNDS,
  //|--------+--------+--------+--------+--------+--------+--------+---------|  |--------+--------+--------+-------+--------+--------+--------+---------|
     KC_LSFT, KC_AT,   KC_TRNS, KC_TRNS,  KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,    KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,KC_TRNS, KC_LBRC, KC_RCBR, KC_TILD,
  //|--------+--------+--------+--------+--------+--------+--------+---------|  |--------+--------+--------+-------+--------+--------+--------+---------|
     KC_LCTL, KC_TRNS, KC_TRNS, KC_TRNS,  KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,    KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,KC_TRNS, KC_LCBR, KC_RCBR, KC_PIPE,
  //|--------+--------+--------+--------+--------+--------+--------+---------|  |--------+--------+--------+-------+--------+--------+--------+---------|
     KC_TAB,  KC_TRNS, KC_TRNS, KC_TRNS,  XXXXXXX, MO(3),  KC_TRNS, KC_TRNS,    KC_TRNS, KC_TRNS, KC_TRNS, XXXXXXX, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS
  //|--------+--------+--------+--------+--------+--------+--------+---------.  ,---------+-------+--------+-------+--------+--------+--------+---------'
  ),
   [3] = LAYOUT(
//,-------------------------------------------------------------------------.    ,-----------------------------------------------------------------------.
     RGB_TOG, RGB_HUI, RGB_SAI, RGB_VAI, REC_RST, KC_TRNS, SCRL_DVI, SCRL_MO,    SCRL_TO, KC_TRNS, KC_TRNS, KC_VOLU,  KC_BRIU, KC_MS_BTN1,KC_MS_UP,KC_MS_BTN2,
  //|--------+--------+--------+--------+--------+--------+--------+---------|  |--------+--------+--------+--------+--------+--------+--------+---------|
     RGB_MOD, RGB_HUD, RGB_SAD, RGB_VAD, REC_SAVE,KC_MS_BTN2,SCRL_DVD, KC_TRNS,  KC_TRNS, KC_TRNS, KC_MS_BTN2,KC_VOLD,KC_BRID, KC_MS_LEFT,KC_MS_D, KC_MS_R,
  //|--------+--------+--------+--------+--------+--------+--------+---------|  |--------+--------+--------+--------+--------+--------+--------+---------|
     KC_TRNS, KC_TRNS,CPI_I100,CPI_D100,KC_TRNS,  KC_MS_BTN2, KC_MS_BTN1,KC_TRNS,KC_TRNS, KC_MS_BTN1,KC_MS_BTN2,KC_MUTE, KC_TRNS, KC_TRNS,KC_MS_WH_UP,KC_MS_WH_DOWN,
  //|--------+--------+--------+--------+--------+--------+--------+---------|  |--------+--------+--------+--------+--------+--------+--------+---------|
     KC_TRNS, KC_TRNS,CPI_I1K, CPI_D1K, XXXXXXX,  KC_TRNS,  KC_TRNS, KC_MS_BTN1, KC_MS_BTN1,KC_TRNS,KC_TRNS, XXXXXXX, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS     
  //|--------+--------+--------+--------+--------+--------+-------+----------.   ,-------+--------+--------+--------+--------+--------+--------+---------'
 )
};
// clang-format on


layer_state_t layer_state_set_user(layer_state_t state) {
    uint8_t layer = get_highest_layer(state);

#ifdef POINTING_DEVICE_AUTO_MOUSE_ENABLE
    switch (get_highest_layer(remove_auto_mouse_layer(state, true))) {
        case 3:
            state = remove_auto_mouse_layer(state, false);
            set_auto_mouse_enable(false);
            break;
        default:
            set_auto_mouse_enable(true);
            break;
    }
#endif

    // ★ レイヤー変更通知（I2C / USB 等）
    ete_on_layer(layer);

    return state;
}

bool encoder_update_user(uint8_t index, bool clockwise) {
    keypos_t key;
    if(index == 0){
     if(clockwise){
         key.row = 4;
         key.col = 0;
       } else {
         key.row = 4;
         key.col = 1;
       }
      
       uint8_t layer = layer_switch_get_layer(key);
       uint8_t keycode = keymap_key_to_keycode(layer,key);
       tap_code16(keycode);
    
    }else if(index == 1){
      if(clockwise){
         key.row = 12;
         key.col = 0;
       } else {
         key.row = 12;
         key.col = 1;
       }
      
       uint8_t layer = layer_switch_get_layer(key);
       uint8_t keycode = keymap_key_to_keycode(layer,key);
       tap_code16(keycode);
}
return false; 
}

void keyboard_post_init_user(void) {
#if POINTING_DEVICE_DEBUG
    debug_enable=true;
#endif
    // debug_matrix=true;
    // debug_keyboard=true;
    // debug_mouse=true;
    rgblight_sethsv(HSV_BLUE);
}


const matrix_row_t matrix_mask[MATRIX_ROWS] = {
    0b00001111, // row 0: cols 0,1,2,3
    0b00001111, // row 1: cols 0,1,2,3
    0b00001111, // row 2: cols 0,1,2,3
    0b00001111, // row 3: cols 0,1,2,3
    0b11110000, // row 4: cols 4,5,6,7
    0b11110000, // row 5: cols 4,5,6,7
    0b11110000, // row 6: cols 4,5,6,7
    0b11110000, // row 7: cols 4,5,6,7
    0b00001111, // row 8: cols 0,1,2,3
    0b00001111, // row 9: cols 0,1,2,3
    0b00001111, // row10: cols 0,1,2,3
    0b00001111, // row11: cols 0,1,2,3
    0b11110000, // row12: cols 4,5,6,7
    0b11111000, // row13: cols 4,5,6,7
    0b11110000, // row14: cols 4,5,6,7
    0b11110000, // row15: cols 4,5,6,7
};
void matrix_init_user(void) {
    ete_init();
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {

    // --- ① 既存のスクロール制御 ---
    switch (keycode) {
        case SCROLL:
            set_scrolling = record->event.pressed;
            break;
        default:
            break;
    }

    // --- ② ETE 側のキー通知 ---
    ete_on_key(keycode, record);

    // --- ③ QMK の通常処理は止めない ---
    return true;
}


void matrix_scan_user(void) {
    ete_tick();
}


//POINTING DEVICE Rightをカーソル移動、Leftをスクロール（Master Right）
report_mouse_t pointing_device_task_combined_user(report_mouse_t left_report, report_mouse_t right_report) {
    left_report.h = left_report.x/4;//除数でスクロールの速度を調整1-4
    left_report.v = left_report.y/4;//除数でスクロールの速度を調整1-4
    left_report.x = 0;
    left_report.y = 0;
    return pointing_device_combine_reports(left_report, right_report);
}