PROJECT = blinker
BUILD_DIR = bin-$(BOARD)

CFILES = main.c
CFILES += delay.c stts75.c

INCLUDES += $(patsubst %,-I%, .)

OPENCM3_DIR=/git/libopencm3

### This section can go to an arch shared rules eventually...
LDSCRIPT = stm32g031.ld
OPENCM3_LIB = opencm3_stm32g0
OPENCM3_DEFS = -DSTM32G0
ARCH_FLAGS = -mthumb -mcpu=cortex-m0plus
OOCD_INTERFACE = stlink
OOCD_TARGET = stm32g0x

include ./rules.mk
