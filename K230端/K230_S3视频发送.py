"""
H题 K230 球检测 + UART位置
K230(GPIO32)→UART→S3(GPIO18)→PID  视频看K230自带LCD
"""
from libs.PipeLine import PipeLine
from libs.YOLO import YOLO11
from libs.Utils import *
from machine import UART, FPIOA
import os, gc, time

kmodel_path = "/sdcard/steel_ball_v4.kmodel"
labels = ["ball"]
model_input_size = [320, 320]
confidence_threshold = 0.4
nms_threshold = 0.45

# 实测像素: 最左28, 中心280, 最右600 (管子左窄右宽, 分段校准)
CENTER_PX = 280
CM_PER_PX_L = 12.5 / (280 - 28)   # 左半: 0.0496 cm/px
CM_PER_PX_R = 12.5 / (600 - 280)  # 右半: 0.0391 cm/px
Y_TOP, Y_BOTTOM = 0, 480

ALPHA = 0.6
LOST_HOLD_FRAMES = 3

UART_BAUD = 460800
UART_TX, UART_RX = 32, 33

_pos_cm, _valid, _lost_count = 0.0, False, 0
_last_ball_cx, _last_ball_cy = 0.0, 0.0
_prev_pos_cm, _prev_time_ms = 0.0, 0
_velocity_cms = 0.0
_fps_filter = [25.0]*10

def _px_to_cm(px):
    if px < CENTER_PX:
        return (px - CENTER_PX) * CM_PER_PX_L
    else:
        return (px - CENTER_PX) * CM_PER_PX_R

fpioa = FPIOA()
fpioa.set_function(UART_TX, FPIOA.UART3_TXD, ie=0, oe=1)
fpioa.set_function(UART_RX, FPIOA.UART3_RXD, ie=1, oe=0)
uart = UART(UART.UART3, baudrate=UART_BAUD)

pl = PipeLine(rgb888p_size=[640,480], display_mode="lcd")
pl.create()
display_size = pl.get_display_size()

yolo = YOLO11(
    task_type="detect", mode="video",
    kmodel_path=kmodel_path, labels=labels,
    rgb888p_size=[640,480], model_input_size=model_input_size,
    display_size=display_size,
    conf_thresh=confidence_threshold, nms_thresh=nms_threshold,
    max_boxes_num=10, debug_mode=0
)
yolo.config_preprocess()

def _filter_ball(candidates):
    passed = []
    for cx, cy, bw, bh, cf in candidates:
        if cf < confidence_threshold: continue
        ratio = bw/bh if bh > 0 else 0
        if not (0.5 <= ratio <= 2.0): continue
        if not (Y_TOP <= cy <= Y_BOTTOM): continue
        passed.append((cx, cy, bw, bh, cf))
    if not passed: return None
    if _valid:
        return min(passed, key=lambda b: (b[0]-_last_ball_cx)**2+(b[1]-_last_ball_cy)**2)
    return max(passed, key=lambda b: b[4])

clock = time.clock()
print("K230 检测+JPEG+UART")

while True:
    os.exitpoint()
    clock.tick()
    tick_now = time.ticks_ms()
    img = pl.get_frame()
    res = yolo.run(img)

    fps = clock.fps()
    _fps_filter.pop(0); _fps_filter.append(fps)
    fps_avg = sum(_fps_filter)/len(_fps_filter)

    candidates = []
    if isinstance(res, list) and len(res) >= 3:
        bboxes, confs = res[0], (res[2] if len(res)>2 and res[2] else [])
        if isinstance(bboxes, list):
            for i, b in enumerate(bboxes):
                if len(b) >= 4:
                    cx = b[0]+b[2]/2.0; cy = b[1]+b[3]/2.0
                    cf = confs[i] if i < len(confs) else 1.0
                    candidates.append((cx, cy, b[2], b[3], cf))

    best = _filter_ball(candidates)
    yolo.draw_result(res, pl.osd_img)

    if best is not None:
        cx, cy, bw, bh, cf = best
        _last_ball_cx, _last_ball_cy = cx, cy
        raw_cm = _px_to_cm(cx) - 0.6  # 修正中心偏移
        raw_cm = max(-12.5, min(12.5, raw_cm))
        _pos_cm = ALPHA*raw_cm + (1.0-ALPHA)*_pos_cm if _valid else raw_cm
        dt = time.ticks_diff(tick_now, _prev_time_ms) / 1000.0
        if dt > 0.001 and _valid:
            _velocity_cms = (_pos_cm - _prev_pos_cm) / dt
        _prev_pos_cm = _pos_cm
        _prev_time_ms = tick_now
        _valid, _lost_count = True, 0
        uart.write("$B,%d\n" % max(-120, min(120, int(_pos_cm*10))))
    else:
        _lost_count += 1
        if _lost_count > LOST_HOLD_FRAMES:
            _valid = False; _velocity_cms = 0.0
            uart.write("$B,999\n")
        else:
            uart.write("$B,%d\n" % max(-120, min(120, int(_pos_cm*10))))

    W, H = 640, 480
    dx = display_size[0]//2 - 30
    pl.osd_img.draw_line(dx, 0, dx, display_size[1], color=(255,0,0), thickness=1)
    pl.osd_img.draw_rectangle(W-90, 2, 88, 26, color=(0,0,0), fill=True)
    pl.osd_img.draw_string_advanced(W-88, 5, 22, "FPS:%d"%int(fps_avg), color=(0,255,0))
    ps = "X:%.0f %+.1fcm"%(_last_ball_cx, _pos_cm) if _valid else "pos: --"
    pl.osd_img.draw_rectangle(2, H-32, 240, 30, color=(0,0,0), fill=True)
    pl.osd_img.draw_string_advanced(5, H-28, 20, ps, color=(0,255,0))
    pl.show_image()
    gc.collect()

yolo.deinit()
pl.destroy()
