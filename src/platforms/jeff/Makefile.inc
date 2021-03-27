CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

ifeq ($(ENABLE_DEBUG), 1)
CFLAGS += -DDEBUG_ME
CFLAGS += -DENABLE_DEBUG
endif

ifeq ($(CONSOLE_NO_AUTO_CRLF), 1)
CFLAGS += -DCONSOLE_NO_AUTO_CRLF
endif

CFLAGS += -mthumb -mcpu=cortex-m0plus \
	-DSAMD -DSAMD21E17 -DBLACKMAGIC -I../libopencm3/include \
	-Iplatforms/samd -msoft-float -ffunction-sections -fdata-sections -MD

LDFLAGS_BOOT := $(LDFLAGS) --specs=nano.specs -mthumb -mcpu=cortex-m0plus -msoft-float \
	-Wl,-Map=mapfile -Wl,-T,platforms/samd/samd.ld -Wl,--gc-sections \
	-L../libopencm3/lib -nostartfiles -lc -lopencm3_samd 
LDFLAGS = $(LDFLAGS_BOOT) -Wl,-Ttext=0x00002000

ifeq ($(ENABLE_DEBUG), 1)
LDFLAGS += --specs=rdimon.specs
else
LDFLAGS += --specs=nosys.specs
endif

VPATH += platforms/samd

SRC +=	cdcacm.c	\
	timing.c        \
	traceswo.o      \
	usbuart.c       \

all:	blackmagic_full.bin blackmagic.bin blackmagic_dfu.bin blackmagic_dfu.hex

blackmagic_dfu.elf: usbdfu.o
	@echo "  LD      $@"
	$(Q)$(CC) $^ -o $@ $(LDFLAGS_BOOT)

blackmagic_full.bin: blackmagic_dfu.bin blackmagic.bin
	@echo "  CAT     $@"
	$(Q)cp blackmagic.bin jeff-$(shell git describe --tags --always --dirty).bin
	$(Q)cat $^ > $@

host_clean:
	$(Q)$(RM) -f blackmagic.bin blackmagic_full.bin blackmagic_dfu blackmagic_dfu.bin blackmagic_dfu.hex jeff*.bin
