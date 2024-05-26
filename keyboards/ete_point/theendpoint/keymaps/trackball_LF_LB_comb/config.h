// Copyright 2023 YMGWorks (@YMGWorks)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

//#define MASTER_RIGHT

#define SPLIT_POINTING_ENABLE
#define POINTING_DEVICE_COMBINED
#define POINTING_DEVICE_ROTATION_180
#define POINTING_DEVICE_INVERT_X_RIGHT
#define POINTING_DEVICE_AUTO_MOUSE_ENABLE
#define AUTO_MOUSE_DEFAULT_LAYER 3
#define AUTO_MOUSE_TIME 500

#define ENCODERS_PAD_A { B4 }
#define ENCODERS_PAD_B { B5 }
#define ENCODER_RESOLUTIONS {3}

#define SPLIT_TRANSACTION_IDS_KB ETE_GET_INFO, ETE_GET_MOTION, ETE_SET_CPI

//#define SPI_DRIVER B6
//#define SPI_SCK_PIN B1
//#define SPI_MISO_PIN B3
//#define SPI_MOSI_PIN B2
//#define POINTING_DEVICE_CS_PIN B6
//#define PMW33XX_CPI 500
//#define PMW33XX_CS_DIVISOR 64