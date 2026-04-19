/*
Copyright 2022 MURAOKA Taro (aka KoRoN, @kaoriya)
Copyright 2023 kushima8 (@kushima8)
Copyright 2023 YMGWorks

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

//////////////////////////////////////////////////////////////////////////////
// Configurations

#ifndef ETE_CPI_DEFAULT
#    define ETE_CPI_DEFAULT 500
#endif

#ifndef ETE_SCROLL_DIV_DEFAULT
#    define ETE_SCROLL_DIV_DEFAULT 4
 // 4: 1/8 (1/2^(n-1))
#endif

#ifndef ETE_REPORTMOUSE_INTERVAL
#    define ETE_REPORTMOUSE_INTERVAL 8 // mouse report rate: 125Hz
#endif

#ifndef ETE_SCROLLBALL_INHIVITOR
#    define ETE_SCROLLBALL_INHIVITOR 80
#endif

#ifndef ETE_SCROLLSNAP_ENABLE
#    define ETE_SCROLLSNAP_ENABLE 0
#endif

#ifndef ETE_SCROLLSNAP_RESET_TIMER
#    define ETE_SCROLLSNAP_RESET_TIMER 100
#endif

#ifndef ETE_SCROLLSNAP_TENSION_THRESHOLD
#    define ETE_SCROLLSNAP_TENSION_THRESHOLD 12
#endif


//////////////////////////////////////////////////////////////////////////////
// Constants

#define ETE_TX_GETINFO_INTERVAL 500
#define ETE_TX_GETINFO_MAXTRY 10
#define ETE_TX_GETMOTION_INTERVAL 4


//////////////////////////////////////////////////////////////////////////////
// Types


typedef struct {
    uint32_t raw;
} ETE_config_t;

// EEPROM bit layout

#define ETE_CONF_CPI_SHIFT       0
#define ETE_CONF_CPI_MASK        0x0000007FUL   // 0..6   (7bit)

#define ETE_CONF_SDIV_SHIFT      7
#define ETE_CONF_SDIV_MASK       0x00000380UL   // 7..9   (3bit)

#define ETE_CONF_SCROLL_SHIFT    10
#define ETE_CONF_SCROLL_MASK     0x0001FC00UL   // 10..16 (7bit)

#define ETE_CONF_CURSOR_SHIFT    17
#define ETE_CONF_CURSOR_MASK     0x00FE0000UL   // 17..23 (7bit)

#define ETE_CONF_INERTIA_SHIFT   24
#define ETE_CONF_INERTIA_MASK    0x7F000000UL   // 24..30 (7bit)

#define ETE_CONF_SWAP_SHIFT      31
#define ETE_CONF_SWAP_MASK       0x80000000UL   // bit31


// getters
static inline uint8_t ete_conf_get_cpi(uint32_t raw) {
    return (uint8_t)((raw & ETE_CONF_CPI_MASK) >> ETE_CONF_CPI_SHIFT);
}

static inline uint8_t ete_conf_get_sdiv(uint32_t raw) {
    return (uint8_t)((raw & ETE_CONF_SDIV_MASK) >> ETE_CONF_SDIV_SHIFT);
}

static inline uint8_t ete_conf_get_scroll_speed(uint32_t raw) {
    return (uint8_t)((raw & ETE_CONF_SCROLL_MASK) >> ETE_CONF_SCROLL_SHIFT);
}

static inline uint8_t ete_conf_get_cursor_speed(uint32_t raw) {
    return (uint8_t)((raw & ETE_CONF_CURSOR_MASK) >> ETE_CONF_CURSOR_SHIFT);
}

static inline bool ete_conf_get_swap_lr(uint32_t raw) {
    return ((raw & ETE_CONF_SWAP_MASK) != 0);
}

static inline uint8_t ete_conf_get_inertia(uint32_t raw) {
    return (raw & ETE_CONF_INERTIA_MASK) >> ETE_CONF_INERTIA_SHIFT;
}

// setters
static inline uint32_t ete_conf_set_cpi(uint32_t raw, uint8_t v) {
    raw &= ~ETE_CONF_CPI_MASK;
    raw |= (((uint32_t)v << ETE_CONF_CPI_SHIFT) & ETE_CONF_CPI_MASK);
    return raw;
}

static inline uint32_t ete_conf_set_sdiv(uint32_t raw, uint8_t v) {
    raw &= ~ETE_CONF_SDIV_MASK;
    raw |= (((uint32_t)v << ETE_CONF_SDIV_SHIFT) & ETE_CONF_SDIV_MASK);
    return raw;
}

static inline uint32_t ete_conf_set_scroll_speed(uint32_t raw, uint8_t v) {
    raw &= ~ETE_CONF_SCROLL_MASK;
    raw |= (((uint32_t)v << ETE_CONF_SCROLL_SHIFT) & ETE_CONF_SCROLL_MASK);
    return raw;
}

static inline uint32_t ete_conf_set_cursor_speed(uint32_t raw, uint8_t v) {
    raw &= ~ETE_CONF_CURSOR_MASK;
    raw |= (((uint32_t)v << ETE_CONF_CURSOR_SHIFT) & ETE_CONF_CURSOR_MASK);
    return raw;
}

static inline uint32_t ete_conf_set_swap_lr(uint32_t raw, bool v) {
    raw &= ~ETE_CONF_SWAP_MASK;
    if (v) {
        raw |= ETE_CONF_SWAP_MASK;
    }
    return raw;
}

static inline uint32_t ete_conf_set_inertia(uint32_t raw, uint8_t v) {
    if (v > 127) v = 127;
    raw &= ~ETE_CONF_INERTIA_MASK;
    raw |= ((uint32_t)v << ETE_CONF_INERTIA_SHIFT);
    return raw;
}

typedef struct {
    uint8_t ballcnt; // count of balls: support only 0 or 1, for now
} ETE_info_t;

typedef struct {
    int16_t x;
    int16_t y;
} ETE_motion_t;

typedef uint8_t ETE_cpi_t;

typedef struct {
    bool this_have_ball;
    bool that_enable;
    bool that_have_ball;
	bool negotiated;

    ETE_motion_t this_motion;
    ETE_motion_t that_motion;

    uint8_t cpi_value;
    bool    cpi_changed;

    bool     scroll_mode;
    uint32_t scroll_mode_changed;
    uint8_t  scroll_div;

    uint32_t scroll_snap_last;
    int8_t   scroll_snap_tension_h;

    uint16_t       last_kc;
    keypos_t       last_pos;
    report_mouse_t last_mouse;
} ETE_t;

typedef enum {
    ETE_ADJUST_PENDING   = 0,
    ETE_ADJUST_PRIMARY   = 1,
    ETE_ADJUST_SECONDARY = 2,
} ETE_adjust_t;

//////////////////////////////////////////////////////////////////////////////
// Exported values (touch carefully)

extern ETE_t ETE;

//////////////////////////////////////////////////////////////////////////////
// Public API functions

/// ETE_get_scroll_mode gets current scroll mode.
bool ETE_get_scroll_mode(void);

/// ETE_set_scroll_mode modify scroll mode.
void ETE_set_scroll_mode(bool mode);

// TODO: document
uint8_t ETE_get_scroll_div(void);

// TODO: document
void ETE_set_scroll_div(uint8_t div);

// TODO: document
uint8_t ETE_get_cpi(void);

// TODO: document
void ETE_set_cpi(uint8_t cpi);

bool ete_process_ball_key(uint16_t keycode, keyrecord_t *record);