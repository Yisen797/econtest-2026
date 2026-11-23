#!/bin/bash
# ============================================================================
# V1 寻迹小车 — 编译 & 烧录脚本
# 用法: 在 Git Bash 中运行
#   ./build.sh         编译
#   ./build.sh flash   编译+烧录
# ============================================================================
set -e
export PATH="/e/bin:$PATH"

SDK="/path/to/tools/TIMSPM0工具链/mspm0_sdk_2_11_00_07"
SRC="src"
BLD="build"
FW="%TOOLS%/mspm0_lib/fw.bin"

I="-I$SRC -I$SDK/source -I$SDK/source/ti/devices/msp/m0p \
   -I$SDK/source/third_party/CMSIS/Core/Include \
   -I$SDK/source/ti/driverlib -I$SDK/source/ti/driverlib/m0p \
   -isystem /path/to/tools/TIMSPM0工具链/arm-none-eabi/include"

F="-mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -Os \
   -ffunction-sections -fdata-sections -fno-common \
   -std=gnu11 -D__MSPM0G3507__ -DDEVICE_MSPM0G3507 -Dgcc"

LDFLAGS="-mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -nostdlib -Wl,--gc-sections"
LIBS="%TOOLS%/mspm0_lib/driverlib.a \
      %TOOLS%/mspm0_lib/thumb/v6-m/nofp/libc_nano.a \
      /e/lib/gcc/arm-none-eabi/15.2.1/thumb/v6-m/nofp/libgcc.a"
LDS="%TOOLS%/mspm0_lib/mspm0g3507.lds"
STARTUP="$SDK/source/ti/devices/msp/m0p/startup_system_files/gcc/startup_mspm0g350x_gcc.c"

rm -rf $BLD && mkdir $BLD

echo "=== V1 寻迹小车 ==="
arm-none-eabi-gcc $F $I -c $SRC/main.c -o $BLD/main.o
arm-none-eabi-gcc $F $I -c $SRC/mpu6050_drv.c -o $BLD/mpu.o
arm-none-eabi-gcc $F $I -c $SRC/oled_ssd1306.c -o $BLD/oled.o
arm-none-eabi-gcc $F $I -c $SRC/sensor.c -o $BLD/sensor.o
arm-none-eabi-gcc $F $I -c "$STARTUP" -o $BLD/startup.o
echo "  compile OK (5 files)"

arm-none-eabi-gcc $BLD/main.o $BLD/mpu.o $BLD/oled.o $BLD/sensor.o $BLD/startup.o \
    $LDFLAGS -T $LDS $LIBS -o $BLD/tracker_v1.elf
echo "  link OK"

arm-none-eabi-objcopy -O binary $BLD/tracker_v1.elf $FW
echo "  fw.bin OK"
arm-none-eabi-size $BLD/tracker_v1.elf

if [ "$1" = "flash" ]; then
    echo ""
    echo "=== J-Link ==="
    printf 'device MSPM0G3507\nsi SWD\nspeed 4000\nr\nh\nsleep 100\nerase 0x00000000,0x0001FFFF\nloadbin %s 0x00000000\nr\ng\nq\n' "$FW" > $BLD/_f.jl
    "/path/to/tools/J-Link资源/JLink_V810e/JLink.exe" -CommanderScript $BLD/_f.jl
    echo "  flash OK"
fi
