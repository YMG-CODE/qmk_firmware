ENCODER_ENABLE = yes
MOUSEKEY_ENABLE = yes

# Optical sensor driver for trackball.
POINTING_DEVICE_ENABLE = yes
POINTING_DEVICE_SPLITS += yes



# Include common library
SRC += lib_combine/ETE/ETE.c
SRC += drivers/pmw3360/pmw3360.c
POINTING_DEVICE_DRIVER =custom

QUANTUM_LIB_SRC += spi_master.c # Optical sensor use SPI to communicate

I2C_ENABLE = yes
I2C_DRIVER_REQUIRED = yes
#SRC += M5meter.c
RAW_HID_ENABLE = yes

CONSOLE_ENABLE = yes

SRC += ete_common.c

ETE_ENABLE_SOLENOID = yes


#EXTRAFLAGS += -DETE_ENABLE_RAW
EXTRAFLAGS += -DETE_ENABLE_I2C
EXTRAFLAGS += -DETE_ENABLE_SOLENOID