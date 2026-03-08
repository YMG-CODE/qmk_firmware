VIA_ENABLE = yes

LTO_ENABLE = yes
RGBLIGHT_ENABLE = yes        # Enable keyboard RGB underglow
RGBLIGHT_LIMIT_VAL = 255

#BOARD = GENERIC_RP_RP2040
#define SPLIT_USB_TIMEOUT 7000 //Default 2000
#define SPLIT_USB_TIMEOUT_POLL 25 //Default 10

WS2812_DRIVER = vendor # RP2040用に追加
SERIAL_DRIVER = vendor # RP2040用に分割キーボードで追加
WS2812_DRIVER = vendor
RGB_MATRIX_DRIVER = WS2812


ENCODER_ENABLE = yes

# Build Options

# Disable other features to squeeze firmware size
SPACE_CADET_ENABLE = no
MAGIC_ENABLE = yes
MUSIC_ENABLE = yes
BACKLIGHT_ENABLE = no       # Enable keyboard backlight functionality
