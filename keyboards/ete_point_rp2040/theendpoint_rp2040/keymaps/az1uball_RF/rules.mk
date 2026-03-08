POINTING_DEVICE_SPLITS += yes
POINTING_DEVICE_ENABLE = yes
POINTING_DEVICE_DRIVER = pimoroni_trackball
I2C_DRIVER_REQUIRED = yes

QUANTUM_LIB_SRC += spi_master.c # Optical sensor use SPI to communicate
MOUSEKEY_ENABLE = yes

# Include common library
SRC += lib_combine/ETE/ETE.c
SRC += drivers/pmw3360/pmw3360.c