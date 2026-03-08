// Copyright 2023 YMGWorks (@YMGWorks)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#define DISABLE_SYNC_TIMER


#define BOOTMAGIC_LITE_ROW 0
#define BOOTMAGIC_LITE_COLUMN 1

#define RP2040_BOOTLOADER_DOUBLE_TAP_RESET
#define RP2040_BOOTLOADER_DOUBLE_TAP_RESET_TIMEOUT 500U

#define MATRIX_MASKED

// Encoder parameters
#define ENCODERS_PAD_A { GP29 }
#define ENCODERS_PAD_B { GP28 }
#define ENCODER_RESOLUTIONS {3}

// LED parameters
#define WS2812_PIO_USE_PIO1 
#define WS2812_DI_PIN GP0
#ifdef RGBLIGHT_ENABLE
 #define RGBLIGHT_LAYERS
 #define RGBLED_NUM 20
 #define RGBLED_SPLIT {10, 10}
 #define RGBLIGHT_SPLIT
#endif

/*== lighting parameters ==*/
#    define RGBLIGHT_HUE_STEP 8
#    define RGBLIGHT_SAT_STEP 8
#    define RGBLIGHT_VAL_STEP 8
#    define RGBLIGHT_LIMIT_VAL 255
#    undef RGBLIGHT_SLEEP  /* If defined, the RGB lighting will be switched off when the host goes to sleep */
/*== all animations enable ==*/
#    define RGBLIGHT_ANIMATIONS
/*== or choose animations ==*/
#    define RGBLIGHT_EFFECT_BREATHING
#    define RGBLIGHT_EFFECT_RAINBOW_MOOD
#    define RGBLIGHT_EFFECT_RAINBOW_SWIRL
#    define RGBLIGHT_EFFECT_SNAKE
#    define RGBLIGHT_EFFECT_KNIGHT
#    define RGBLIGHT_EFFECT_CHRISTMAS
#    define RGBLIGHT_EFFECT_STATIC_GRADIENT
#    define RGBLIGHT_EFFECT_RGB_TEST
#    define RGBLIGHT_EFFECT_ALTERNATING
/*== customize breathing effect ==*/

/*==== (DEFAULT) use fixed table instead of exp() and sin() ====*/
#    define RGBLIGHT_BREATHE_TABLE_SIZE 256      // 256(default) or 128 or 64
/*==== use exp() and sin() ====*/
#    define RGBLIGHT_EFFECT_BREATHE_CENTER 1.85  // 1 to 2.7
#    define RGBLIGHT_EFFECT_BREATHE_MAX    255   // 0 to 255


// To squeeze firmware size
#undef LOCKING_SUPPORT_ENABLE
#undef LOCKING_RESYNC_ENABLE
#define NO_MUSIC_MODE
#if !defined(LAYER_STATE_8BIT) && !defined(LAYER_STATE_16BIT) && !defined(LAYER_STATE_32BIT)
#    define LAYER_STATE_8BIT
#endif

// To squeeze firmware size_optional
/* disable debug print */
//#define NO_DEBUG
/* disable print */
//#define NO_PRINT
/* disable action features */
//#define NO_ACTION_LAYER
//#define NO_ACTION_TAPPING
//#define NO_ACTION_ONESHOT
