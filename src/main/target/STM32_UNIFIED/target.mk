
ifeq ($(TARGET), $(filter $(TARGET), STM32F405 STM32F405_OSD))
F405_TARGETS += $(TARGET)
endif

ifeq ($(TARGET), $(filter $(TARGET), STM32F411 STM32F411_OSD))
F411_TARGETS += $(TARGET)
endif

ifeq ($(TARGET), $(filter $(TARGET), STM32F7X2 STM32F7X2_OSD))
F7X2RE_TARGETS += $(TARGET)
endif

ifeq ($(TARGET), $(filter $(TARGET), STM32F745 STM32F745_OSD))
F7X5XG_TARGETS += $(TARGET)
endif

ifeq ($(TARGET), $(filter $(TARGET), STM32G47X STM32G47X_OSD))
G47X_TARGETS += $(TARGET)
endif

ifeq ($(TARGET), $(filter $(TARGET), STM32H743 STM32H743_OSD))
H743xI_TARGETS += $(TARGET)
endif

# Use a full block (16 kB) of flash for custom defaults - with 1 MB flash we have more than we know how to use anyway
ifeq ($(TARGET), $(filter $(TARGET), STM32F405 STM32F405_OSD STM32F745 STM32F745_OSD STM32H743 STM32H743_OSD))
CUSTOM_DEFAULTS_EXTENDED = yes
endif

ifeq ($(TARGET), $(filter $(TARGET), STM32G47X STM32G47X_OSD))
FEATURES += VCP SDCARD_SPI ONBOARDFLASH
else
FEATURES += VCP SDCARD_SPI SDCARD_SDIO ONBOARDFLASH
endif

TARGET_SRC = \
    $(addprefix drivers/accgyro/,$(notdir $(wildcard $(SRC_DIR)/drivers/accgyro/*.c))) \
    $(ROOT)/lib/main/BoschSensortec/BMI270-Sensor-API/bmi270_maximum_fifo.c \
    $(addprefix drivers/barometer/,$(notdir $(wildcard $(SRC_DIR)/drivers/barometer/*.c))) \
    $(addprefix drivers/compass/,$(notdir $(wildcard $(SRC_DIR)/drivers/compass/*.c)))

ifeq ($(TARGET), $(filter $(TARGET), STM32F405_OSD STM32F411_OSD STM32F7X2_OSD STM32F745_OSD STM32G47X_OSD STM32H743_OSD))
VARIANT_SRC = \
    drivers/max7456.c \
    drivers/vtx_rtc6705.c \
    drivers/vtx_rtc6705_soft_spi.c
endif
