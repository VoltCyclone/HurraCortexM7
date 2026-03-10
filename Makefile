TOOLCHAIN = /Users/ramseymcgrath/.platformio/packages/toolchain-gccarmnoneeabi-teensy/bin
CC      = $(TOOLCHAIN)/arm-none-eabi-gcc
OBJCOPY = $(TOOLCHAIN)/arm-none-eabi-objcopy
SIZE    = $(TOOLCHAIN)/arm-none-eabi-size

TARGET  = firmware

MCU_FLAGS = -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb

# Pass UART=1 on command line to enable debug UART output (e.g. make UART=1)
UART ?= 0
# Override UART baud rate (e.g. make UART_BAUD=921600)
UART_BAUD ?= 2000000
# Pass TFT=1 to enable TFT display (e.g. make TFT=1)
TFT ?= 1
# TFT driver: 1=ST7735 (128x160), 3=ILI9341 (240x320)
TFT_DRIVER ?= 1
# Pass TOUCH=1 to enable FT6206 capacitive touch (requires TFT_DRIVER=3)
TOUCH ?= 0
# Pass UART_AUTOBAUD=1 to detect baud from incoming RX data at boot
UART_AUTOBAUD ?= 0
# Pass NET=1 to enable Ethernet (KMBox Net UDP protocol, replaces UART commands)
NET ?= 0

DEFINES = -DARDUINO_TEENSY41 -D__IMXRT1062__ -DF_CPU=816000000 -DUART_ENABLED=$(UART) \
          -DUART_BAUD=$(UART_BAUD) -DTFT_ENABLED=$(TFT) -DTFT_DRIVER=$(TFT_DRIVER) \
          -DTOUCH_ENABLED=$(TOUCH) -DUART_AUTOBAUD=$(UART_AUTOBAUD) \
          -DNET_ENABLED=$(NET)

CFLAGS = $(MCU_FLAGS) $(DEFINES) \
         -Os -Wall -Wno-unused-variable \
         -ffunction-sections -fdata-sections \
         -Iinclude -Isrc

LDFLAGS = $(MCU_FLAGS) \
          -Tcore/imxrt1062_t41.ld \
          -Wl,--gc-sections \
          --specs=nano.specs --specs=nosys.specs

CORE_SRC = core/startup.c core/bootdata.c
SRC      = src/main.c src/uart.c src/usb_host.c src/usb_device.c src/desc_capture.c \
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
$(HOT_SRC): CFLAGS := $(subst -Os,-O2,$(CFLAGS))

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
