LIBOPENCM3_DIR = libopencm3

PREFIX=arm-none-eabi
CC=$(PREFIX)-gcc
OBJCOPY=$(PREFIX)-objcopy
AR=$(PREFIX)-ar

COMMON_FLAGS=-mcpu=cortex-m3 -mthumb -mfix-cortex-m3-ldrd -msoft-float -Wall
CFLAGS=$(COMMON_FLAGS) -Os -std=gnu99 -I. -I$(LIBOPENCM3_DIR)/include -DSTM32F1 -fno-common -ffunction-sections -fdata-sections \
	-Wstrict-prototypes -Wundef -Wextra -Wshadow -Wredundant-decls
LDFLAGS=$(COMMON_FLAGS) -lc -nostartfiles -Wl,--gc-sections -Wl,--print-gc-sections

PHONY:all

all: main.bin

%.o : %.c
	$(CC) $(CFLAGS) -c $< -o $@

LIBOPENCM3_OBJS = $(LIBOPENCM3_DIR)/lib/stm32/f1/gpio.o \
		$(LIBOPENCM3_DIR)/lib/stm32/f1/rcc.o \
		$(LIBOPENCM3_DIR)/lib/stm32/f1/pwr.o \
		$(LIBOPENCM3_DIR)/lib/stm32/f1/timer.o \
		$(LIBOPENCM3_DIR)/lib/stm32/f1/flash.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/gpio_common_all.o \
 		$(LIBOPENCM3_DIR)/lib/stm32/common/usart_common_all.o \
 		$(LIBOPENCM3_DIR)/lib/stm32/common/usart_common_f124.o \
 		$(LIBOPENCM3_DIR)/lib/stm32/common/rcc_common_all.o \
 		$(LIBOPENCM3_DIR)/lib/stm32/common/pwr_common_all.o \
 		$(LIBOPENCM3_DIR)/lib/stm32/common/i2c_common_all.o \
 		$(LIBOPENCM3_DIR)/lib/cm3/vector.o \
		$(LIBOPENCM3_DIR)/lib/cm3/systick.o \
 		$(LIBOPENCM3_DIR)/lib/cm3/nvic.o \
 		$(LIBOPENCM3_DIR)/lib/cm3/sync.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/timer_common_all.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/flash_common_f01.o \
		$(LIBOPENCM3_DIR)/lib/stm32/can.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/iwdg_common_all.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/exti_common_all.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/spi_common_all.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/spi_common_l1f124.o

main.bin: main.o ring.o $(LIBOPENCM3_OBJS)
	$(CC) $^ -L. -Tbluepill.ld $(LDFLAGS) -Wl,-Map=main.map -o main.elf
	$(OBJCOPY) -O binary main.elf $@

flash: main.bin
	./stm32loader.py -p /dev/ttyUSB0 -b 115200 -V -e -w -v $<

clean:
	rm -rf *.bin *.map $(LIBOPENCM3_OBJS) *.o *.d *.elf

