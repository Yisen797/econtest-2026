# OpenMV V10 硬件JPEG — sensor.JPEG直出, QVGA, 921600baud, 无需软件压缩
import sensor, time
from pyb import UART

sensor.reset()
sensor.set_pixformat(sensor.JPEG)   # 硬件JPEG: OV5640直接编码, 零CPU开销
sensor.set_framesize(sensor.QQVGA)  # 160x120, ~1KB/帧, 可达30fps
sensor.set_auto_exposure(True)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.skip_frames(time=1500)

uart = UART(3, 460800)

while True:
    img = sensor.snapshot()
    data = img.bytearray()  # 直接取JPEG字节, 无需compress
    n = len(data)
    if n > 0:
        # 分块256B发送, 避免DMA冲突
        for i in range(0, n, 512):
            uart.write(data[i:i+512])
            time.sleep_us(100)
