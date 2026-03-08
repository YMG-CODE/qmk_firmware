POINTING_DEVICE_SPLITS += yes
POINTING_DEVICE_ENABLE = yes

# Optical sensor driver for trackball.
POINTING_DEVICE_DRIVER = pimoroni_trackball

MOUSEKEY_ENABLE = yes
ENCODER_ENABLE = yes

# Include common library
SRC += lib_unuseoled/ETE/ETE.c

I2C_ENABLE = yes
I2C_DRIVER_REQUIRED = yes

RAW_HID_ENABLE = yes
CONSOLE_ENABLE = yes

SRC += ete_common.c

ETE_ENABLE_SOLENOID = yes

EXTRAFLAGS += -DETE_ENABLE_RAW
EXTRAFLAGS += -DETE_ENABLE_I2C
EXTRAFLAGS += -DETE_ENABLE_SOLENOID