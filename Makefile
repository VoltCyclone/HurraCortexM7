TOOLCHAIN = /Users/ramseymcgrath/.platformio/packages/toolchain-gccarmnoneeabi-teensy/bin
CC      = $(TOOLCHAIN)/arm-none-eabi-gcc
OBJCOPY = $(TOOLCHAIN)/arm-none-eabi-objcopy
SIZE    = $(TOOLCHAIN)/arm-none-eabi-size

TARGET = firmware

MCU_FLAGS = -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb

# Protocol selector: 'hurra' (binary, TinyFrame, default) or 'ferrum' (ASCII).
PROTOCOL ?= hurra

# UART baud for kmbox <-> host link (LPUART3, Teensy pins 16/17, no flow control).
# Hurra defaults to 4 Mbps (its design target, matching the hurra-app/hurra-bridge
# default); Ferrum keeps 115200, the power-on default the Ferrum spec mandates.
# Override either with `make CMD_BAUD=N`.
ifeq ($(PROTOCOL),hurra)
  CMD_BAUD ?= 4000000
  PROTO_DEF = -DPROTOCOL_HURRA
  PROTO_SRC = src/hurra.c src/third_party/TinyFrame/TinyFrame.c
else ifeq ($(PROTOCOL),ferrum)
  CMD_BAUD ?= 115200
  PROTO_DEF = -DPROTOCOL_FERRUM
  PROTO_SRC = src/ferrum.c
else
  $(error PROTOCOL must be 'hurra' or 'ferrum')
endif

# F_CPU: core clock. 600 MHz is the part's rated max; this board runs
# overclocked. 912 MHz needs ~1525 mV core (set_arm_clock interpolates it,
# capped at the 1575 mV silicon max). IPG = F_CPU/4 = 228 MHz stays a whole
# MHz so the GPT2 1 µs tick and LED scale (both F_CPU-derived) remain exact.
# Above ~864 MHz exceeds NXP's 1300 mV recommended limit — trades silicon
# lifetime for clock. Drop to 816000000 to back off; the tempmon thermal
# throttle (main.c) guards against runaway heat at any of these.
F_CPU ?= 912000000
DEFINES = -DARDUINO_TEENSY_MICROMOD -D__IMXRT1062__ -DF_CPU=$(F_CPU) \
          -DCMD_BAUD=$(CMD_BAUD) $(PROTO_DEF)

CFLAGS = $(MCU_FLAGS) $(DEFINES) \
         -Os -Wall -Wno-unused-variable \
         -ffunction-sections -fdata-sections \
         -flto -fsingle-precision-constant \
         -Iinclude -Isrc -Isrc/third_party/TinyFrame

LDFLAGS = $(MCU_FLAGS) \
          -Tcore/imxrt1062_mm.ld \
          -Wl,--gc-sections \
          -flto -fuse-linker-plugin \
          --specs=nano.specs --specs=nosys.specs

CORE_SRC = core/startup.c core/bootdata.c
SRC = src/main.c src/usb_host.c src/usb_device.c src/desc_capture.c \
      src/kmbox.c src/humanize.c src/actions.c src/led.c \
      $(PROTO_SRC)

OBJ = $(CORE_SRC:.c=.o) $(SRC:.c=.o)

all: $(TARGET).hex
	@$(SIZE) $(TARGET).elf

$(TARGET).elf: $(OBJ)
	$(CC) $(LDFLAGS) -o $@ $^ -lm

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# Hot-path sources get -O2 instead of -Os. main.c is included because the
# central poll loop (PIT tick dispatch, EP polling, merge/send) lives there.
HOT_SRC = src/main.o src/usb_host.o src/usb_device.o src/kmbox.o \
          src/humanize.o src/actions.o
ifeq ($(PROTOCOL),hurra)
  HOT_SRC += src/hurra.o src/third_party/TinyFrame/TinyFrame.o
else
  HOT_SRC += src/ferrum.o
endif
$(HOT_SRC): CFLAGS := $(subst -Os,-O2,$(CFLAGS)) -ffast-math

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

flash: $(TARGET).hex
	teensy_loader_cli --mcu=TEENSY_MICROMOD -w -v $(TARGET).hex

clean:
	rm -f $(OBJ) $(TARGET).elf $(TARGET).hex

.PHONY: all flash clean

# Host-native unit tests (no cross-compile). humanize.c must stay free of
# hardware headers behind HUMANIZE_HOSTTEST so it builds with system gcc.
.PHONY: test
test:
	cc -std=c11 -O2 -DHUMANIZE_HOSTTEST -Isrc -o /tmp/humanize_test \
	   test/humanize_test.c src/humanize.c -lm
	/tmp/humanize_test
	cc -std=c11 -O2 -Isrc -o /tmp/motion_test \
	   test/motion_test.c src/actions.c -lm
	/tmp/motion_test
