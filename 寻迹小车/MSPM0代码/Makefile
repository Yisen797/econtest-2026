# ============================================================================
# MSPM0G3507 寻迹小车 — V1 Makefile
# ============================================================================
# 工具链: ARM GCC 15.2.1 (%ARM_GCC%/bin/arm-none-eabi-)
# SDK:    MSPM0 SDK 2.11.00.07
# 库:     %TOOLS%/mspm0_lib/
# ============================================================================

CROSS   := %ARM_GCC%/bin/arm-none-eabi-
CC      := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
SIZE    := $(CROSS)size

CPU     := -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft
OPT     := -Os

MSPM0_SDK   := %TOOLS%/TIMSPM0工具链/mspm0_sdk_2_11_00_07
SDK_SOURCE  := $(MSPM0_SDK)/source
SDK_DEVICE  := $(SDK_SOURCE)/ti/devices/msp/m0p
SDK_CMSIS   := $(SDK_SOURCE)/third_party/CMSIS/Core/Include
SDK_DRVLIB  := $(SDK_SOURCE)/ti/driverlib
LIBC_SYSROOT := %TOOLS%/TIMSPM0工具链/arm-none-eabi
LIBDIR       := %TOOLS%/mspm0_lib

PROJECT     := tracker_v1
SRC_DIR     := src
BUILD_DIR   := build

# V1 源文件
C_SOURCES   := $(SRC_DIR)/main.c \
               $(SRC_DIR)/mpu6050_drv.c \
               $(SRC_DIR)/oled_ssd1306.c \
               $(SRC_DIR)/sensor.c
STARTUP_SRC := $(SDK_DEVICE)/startup_system_files/gcc/startup_mspm0g350x_gcc.c

C_OBJECTS   := $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(C_SOURCES))
C_OBJECTS   += $(BUILD_DIR)/startup.o

DEFINES := -D__MSPM0G3507__ -DDEVICE_MSPM0G3507 -Dgcc

INCLUDES := -I$(SRC_DIR) \
            -I$(SDK_SOURCE) \
            -I$(SDK_DEVICE) \
            -I$(SDK_CMSIS) \
            -I$(SDK_DRVLIB) \
            -isystem $(LIBC_SYSROOT)/include

CFLAGS := $(CPU) $(OPT) -g3 -Wall \
          -ffunction-sections -fdata-sections -fno-common \
          -std=gnu11 $(DEFINES) $(INCLUDES)

LDFLAGS := $(CPU) -nostdlib -Wl,--gc-sections \
           -T $(LIBDIR)/mspm0g3507.lds
LIBS    := $(LIBDIR)/driverlib.a \
           $(LIBDIR)/thumb/v6-m/nofp/libc_nano.a \
           /e/lib/gcc/arm-none-eabi/15.2.1/thumb/v6-m/nofp/libgcc.a

ELF := $(BUILD_DIR)/$(PROJECT).elf
BIN := $(BUILD_DIR)/$(PROJECT).bin

# J-Link 烧录
JLINK := %TOOLS%/J-Link资源/JLink_V810e/JLink.exe
FW    := %TOOLS%/mspm0_lib/fw.bin

.PHONY: all clean flash

all: $(BUILD_DIR) $(ELF) $(BIN)
	@echo "============================================"
	@echo " BUILD SUCCESS"
	@echo "============================================"
	$(SIZE) $(ELF)
	@echo "  flash: make flash"

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@echo "  CC    $<"
	@$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/startup.o: $(STARTUP_SRC)
	@echo "  CC    startup"
	@$(CC) $(CFLAGS) -c $< -o $@

$(ELF): $(C_OBJECTS)
	@echo "  LD    $(ELF)"
	@$(CC) $(C_OBJECTS) $(LDFLAGS) $(LIBS) -o $@

$(BIN): $(ELF)
	@echo "  BIN   $(BIN)"
	@$(OBJCOPY) -O binary $< $@
	@cp $(BIN) $(FW)

flash: $(BIN)
	@echo "Flashing via J-Link..."
	@cp $(BIN) $(FW)
	@printf 'device MSPM0G3507\nsi SWD\nspeed 4000\nr\nh\nerase 0x00000000,0x0001FFFF\nloadbin $(FW) 0x00000000\nr\ng\nq\n' > $(BUILD_DIR)/_flash.jlink
	@"$(JLINK)" -CommanderScript $(BUILD_DIR)/_flash.jlink

clean:
	@rm -rf $(BUILD_DIR)
	@echo "Cleaned."
