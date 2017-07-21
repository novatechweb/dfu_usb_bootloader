CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

CFLAGS += -Istm32/include -mcpu=cortex-m3 -mthumb \
	-DSTM32F1 -I../libopencm3/include \
	-Iplatforms/stm32

LDFLAGS_BOOT := $(LDFLAGS) -lopencm3_stm32f1 -Wl,--defsym,_stack=0x20005000 \
	-Wl,-T,platforms/stm32/stm32f1_dfu.ld -nostartfiles -lc -lnosys \
	-Wl,-Map=mapfile -mthumb -mcpu=cortex-m3 -Wl,-gc-sections \
	-L../libopencm3/lib

VPATH += platforms/stm32

all:	stm32f1_dfu.bin stm32f1_dfu.hex

stm32f1_dfu: i2c_tools.o usbdfu.o dfucore.o dfu_f1.o
	$(CC) $^ -o $@ $(LDFLAGS_BOOT)

stm32f1_dfu.bin:	stm32f1_dfu
	$(OBJCOPY) -O binary $^ $@

stm32f1_dfu.hex:	stm32f1_dfu
	$(OBJCOPY) -O ihex $^ $@

host_clean:
	-@$(RM) stm32f1_dfu stm32f1_dfu.bin stm32f1_dfu.hex

