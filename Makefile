TOOLCHAIN = /Users/ramseymcgrath/.platformio/packages/toolchain-gccarmnoneeabi-teensy/bin
CC      = $(TOOLCHAIN)/arm-none-eabi-gcc
OBJCOPY = $(TOOLCHAIN)/arm-none-eabi-objcopy
SIZE    = $(TOOLCHAIN)/arm-none-eabi-size

TARGET  = firmware

MCU_FLAGS = -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb

# Pass TFT=1 to enable TFT display (auto-detects ILI9341 vs ST7735 at boot)
TFT ?= 1
# Pass NET=1 to enable Ethernet (KMBox Net UDP protocol, replaces UART commands)
NET ?= 0
# Command UART baud rate — LPUART6 on Teensy pins 0/1
CMD_BAUD ?= 4000000

DEFINES = -DARDUINO_TEENSY41 -D__IMXRT1062__ -DF_CPU=816000000 \
          -DTFT_ENABLED=$(TFT) \
          -DNET_ENABLED=$(NET) -DCMD_BAUD=$(CMD_BAUD)

CFLAGS = $(MCU_FLAGS) $(DEFINES) \
         -Os -Wall -Wno-unused-variable \
         -ffunction-sections -fdata-sections \
         -Iinclude -Isrc

LDFLAGS = $(MCU_FLAGS) \
          -Tcore/imxrt1062_t41.ld \
          -Wl,--gc-sections \
          --specs=nano.specs --specs=nosys.specs

CORE_SRC = core/startup.c core/bootdata.c
SRC      = src/main.c src/usb_host.c src/usb_device.c src/desc_capture.c \
           src/kmbox.c src/humanize.c src/smooth.c src/ferrum.c src/makcu.c \
           src/tft.c src/tft_display.c src/st7735.c src/ili9341.c src/ft6206.c \
           src/font6x8.c \
           src/enet.c src/udp.c src/kmnet.c

OBJ = $(CORE_SRC:.c=.o) $(SRC:.c=.o)

BRIDGE_BUILD = uart_bridge/build

all: $(TARGET).hex bridge
	@$(SIZE) $(TARGET).elf

$(TARGET).elf: $(OBJ)
	$(CC) $(LDFLAGS) -o $@ $^ -lm

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# Hot-path sources get -O2 instead of -Os for better inlining/unrolling
HOT_SRC = src/usb_host.o src/usb_device.o src/kmbox.o src/smooth.o src/humanize.o \
          src/enet.o src/kmnet.o
$(HOT_SRC): CFLAGS := $(subst -Os,-O2,$(CFLAGS)) -ffast-math

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

bridge: $(BRIDGE_BUILD)/uart_bridge.uf2

$(BRIDGE_BUILD)/uart_bridge.uf2: uart_bridge/main.c uart_bridge/CMakeLists.txt
	@mkdir -p $(BRIDGE_BUILD)
	cd $(BRIDGE_BUILD) && cmake .. && $(MAKE)

flash: $(TARGET).hex
	teensy_loader_cli --mcu=TEENSY41 -w -v $(TARGET).hex

flash-bridge: $(BRIDGE_BUILD)/uart_bridge.uf2
	@echo "Copy $(BRIDGE_BUILD)/uart_bridge.uf2 to the RP2350 (mount as USB drive)"

clean:
	rm -f $(OBJ) $(TARGET).elf $(TARGET).hex
	rm -rf $(BRIDGE_BUILD)

.PHONY: all flash flash-bridge bridge clean
