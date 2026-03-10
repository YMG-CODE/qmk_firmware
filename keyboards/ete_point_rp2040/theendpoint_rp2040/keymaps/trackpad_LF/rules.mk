ENCODER_ENABLE = yes
MOUSEKEY_ENABLE = yes

POINTING_DEVICE_ENABLE = yes
POINTING_DEVICE_DRIVER = azoteq_iqs5xx

POINTING_DEVICE_SPLITS += yes
I2C_DRIVER_REQUIRED = yes

QUANTUM_LIB_SRC += spi_master.c # Optical sensor use SPI to communicate

# Include common library
SRC += lib_combine/ETE/ETE.c
SRC += drivers/pmw3360/pmw3360.c
SRC += ete_common.c

AZOTEQ_IQS5XX_FILTER_STRENGTH=2
AZOTEQ_IQS5XX_REPORT_RATE=80