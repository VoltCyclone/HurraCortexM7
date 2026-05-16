TOOLCHAIN = /Users/ramseymcgrath/.platformio/packages/toolchain-gccarmnoneeabi-teensy/bin
CC      = $(TOOLCHAIN)/arm-none-eabi-gcc
OBJCOPY = $(TOOLCHAIN)/arm-none-eabi-objcopy
SIZE    = $(TOOLCHAIN)/arm-none-eabi-size

TARGET = firmware

MCU_FLAGS = -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb

# UART baud for kmbox <-> host link.  LPUART6 on Teensy pins 0/1, no flow control.
CMD_BAUD ?= 115200

DEFINES = -DARDUINO_TEENSY_MICROMOD -D__IMXRT1062__ -DF_CPU=816000000 \
          -DCMD_BAUD=$(CMD_BAUD)

CFLAGS = $(MCU_FLAGS) $(DEFINES) \
         -Os -Wall -Wno-unused-variable \
         -ffunction-sections -fdata-sections \
         -Iinclude -Isrc

LDFLAGS = $(MCU_FLAGS) \
          -Tcore/imxrt1062_mm.ld \
          -Wl,--gc-sections \
          --specs=nano.specs --specs=nosys.specs

CORE_SRC = core/startup.c core/bootdata.c
SRC = src/main.c src/usb_host.c src/usb_device.c src/desc_capture.c \
      src/kmbox.c src/humanize.c src/smooth.c src/ferrum.c src/actions.c

OBJ = $(CORE_SRC:.c=.o) $(SRC:.c=.o)

all: $(TARGET).hex
	@$(SIZE) $(TARGET).elf

$(TARGET).elf: $(OBJ)
	$(CC) $(LDFLAGS) -o $@ $^ -lm

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# Hot-path sources get -O2 instead of -Os
HOT_SRC = src/usb_host.o src/usb_device.o src/kmbox.o src/smooth.o \
          src/humanize.o src/ferrum.o src/actions.o
$(HOT_SRC): CFLAGS := $(subst -Os,-O2,$(CFLAGS)) -ffast-math

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

flash: $(TARGET).hex
	teensy_loader_cli --mcu=TEENSY_MICROMOD -w -v $(TARGET).hex

clean:
	rm -f $(OBJ) $(TARGET).elf $(TARGET).hex

.PHONY: all flash clean
