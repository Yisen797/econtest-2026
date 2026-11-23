"""
H题 车载平衡滚球 — K230 视觉定位模块
摄像头固定、垂直俯视摆杆水管(水平走向)，42步进电机控倾角

标定步骤:
  ① 钢球推到 -10cm 刻度线, 记 LCD X 像素值 → 填入 X_LEFT
  ② 钢球推到 +10cm 刻度线, 记 LCD X 像素值 → 填入 X_RIGHT
  ③ 钢球放中心 0 刻度, 若显示不为0, 调 OFFSET_CM
  ④ 两端偏差大时, 调整 CALIB_K 三段系数
"""

from libs.PipeLine import PipeLine
from libs.YOLO import YOLO11
from libs.Utils import *
from machine import UART, FPIOA
import os, sys, gc, time

# ============================================================
# 模型配置
# ============================================================
kmodel_path = "/sdcard/steel_ball_v4.kmodel"
labels = ["ball"]
model_input_size = [320, 320]
confidence_threshold = 0.4    # 置信度阈值 (高→少误检, 低→少漏检)
nms_threshold = 0.45

# ============================================================
# 标定参数 (必调!)
# ============================================================
X_LEFT  = 125   # -10cm 刻度线对应的像素 X 坐标
X_RIGHT = 722   # +10cm 刻度线对应的像素 X 坐标
Y_TOP    = 0    # 摆杆槽上边界 (调试阶段全屏)
Y_BOTTOM = 480  # 摆杆槽下边界 (调试阶段全屏)
OFFSET_CM = 0.0 # 零点校准偏移 (中心显示不为 0 时修正)

# 三段校准系数: 左/中/右段比例修正, 抵消镜头桶形畸变
# 对应区间: [-10,-5), [-5,5], (5,10]
CALIB_K = [1.0, 1.0, 1.0]

# ---- 自动计算 (修改标定后自动联动) ----
_PX_SPAN = X_RIGHT - X_LEFT           # ±10cm 对应的总像素跨度
CM_PER_PX = 20.0 / _PX_SPAN           # 每像素对应厘米数
BALL_DIAM_CM = 1.0                     # 钢球理论直径
BALL_DIAM_PX = BALL_DIAM_CM / CM_PER_PX  # 钢球理论像素宽度
CENTER_PX = (X_LEFT + X_RIGHT) / 2.0  # 刻度中心像素坐标

# ============================================================
# 显示
# ============================================================
display_mode = "lcd"
rgb888p_size = [640, 480]

# ============================================================
# 滤波参数
# ============================================================
ALPHA = 0.6  # 一阶低通系数 (越大响应越快、抖动越大; 越小越平滑、滞后越大)
LOST_HOLD_FRAMES = 3  # 连续丢帧≤此值保持上次位置

# ============================================================
# 极限保护
# ============================================================
LIMIT_CM = 9.5   # 检测位置超出此值触发极限标志

# ============================================================
# UART 配置
# ============================================================
UART_BAUD = 460800
UART_TX = 5   # GPIO5 → S3 GPIO16(RX)
UART_RX = 6   # GPIO6 (预留)

# ============================================================
# 全局状态
# ============================================================
_pos_cm = 0.0          # 当前位置 cm (滤波后)
_pos_raw_cm = 0.0      # 原始位置 cm (无滤波)
_valid = False          # 有效标志
_new_data = False       # 新数据就绪
_lost_count = 0         # 连续丢帧计数
_last_ball_cx = 0.0     # 上一帧球心 X (像素)
_last_ball_cy = 0.0     # 上一帧球心 Y (像素)
_last_conf = 0.0        # 上一帧置信度
_prev_pos_cm = 0.0      # 上周期位置 (速度计算用)
_prev_time_ms = 0       # 上周期时间戳
_velocity_cms = 0.0     # 速度 cm/s
_limit_triggered = False # 极限限位标志
_fps_filter = [25.0] * 10  # FPS 滑动平均 (10帧)

# ============================================================
# 初始化
# ============================================================
fpioa = FPIOA()
fpioa.set_function(UART_TX, FPIOA.UART2_TXD)
fpioa.set_function(UART_RX, FPIOA.UART2_RXD)
uart = UART(UART.UART2, baudrate=UART_BAUD)

pl = PipeLine(rgb888p_size=rgb888p_size, display_mode=display_mode)
pl.create()
display_size = pl.get_display_size()

yolo = YOLO11(
    task_type="detect", mode="video",
    kmodel_path=kmodel_path, labels=labels,
    rgb888p_size=rgb888p_size, model_input_size=model_input_size,
    display_size=display_size,
    conf_thresh=confidence_threshold, nms_thresh=nms_threshold,
    max_boxes_num=10, debug_mode=0
)
yolo.config_preprocess()

# ============================================================
# 坐标换算
# ============================================================
def _px_to_cm(px):
    """像素 X → 厘米 (中心为0, 右正左负)"""
    return (px - CENTER_PX) * CM_PER_PX + OFFSET_CM

def _cm_to_px(cm):
    """厘米 → 像素 X"""
    return (cm - OFFSET_CM) / CM_PER_PX + CENTER_PX

def _get_calib_k(pos_cm):
    """根据位置返回对应段校准系数"""
    if pos_cm < -6.0:
        return CALIB_K[0]
    elif pos_cm <= 6.0:
        return CALIB_K[1]
    else:
        return CALIB_K[2]

def get_ball_pos_cm():
    """返回 (pos_cm, valid, new_data)"""
    global _new_data
    nd = _new_data
    _new_data = False
    return _pos_cm, _valid, nd

def get_ball_error_cm(target_cm):
    """返回 目标位置 - 当前位置 (正值=球偏左, 需向右调)"""
    return target_cm - _pos_cm

def get_ball_velocity_cms():
    """返回钢球瞬时速度 cm/s (带符号)"""
    return _velocity_cms

def clear_new_data():
    """控制端读取后清除新数据标志"""
    global _new_data
    _new_data = False

# ============================================================
# 四级过滤 + 择优
# ============================================================
def _filter_ball(candidates):
    global _last_ball_cx, _last_ball_cy
    passed = []
    n_conf, n_ratio, n_size, n_y = 0, 0, 0, 0

    for cx, cy, bw, bh, cf in candidates:
        if cf < confidence_threshold:
            n_conf += 1; continue
        ratio = bw / bh if bh > 0 else 0
        if not (0.6 <= ratio <= 1.4):
            n_ratio += 1; continue
        if not (BALL_DIAM_PX * 0.25 < bw < BALL_DIAM_PX * 2.0):
            n_size += 1; continue
        if not (Y_TOP <= cy <= Y_BOTTOM):
            n_y += 1; continue
        passed.append((cx, cy, bw, bh, cf))

    if candidates:
        print("flt: conf{} ar{} sz{} y{} -> pass{}".format(
            n_conf, n_ratio, n_size, n_y, len(passed)))

    if not passed:
        return None
    if _valid:
        return min(passed, key=lambda b: (b[0]-_last_ball_cx)**2 + (b[1]-_last_ball_cy)**2)
    return max(passed, key=lambda b: b[4])

# ============================================================
# 主循环
# ============================================================
clock = time.clock()
print("Balance Ball Vision Ready | display:", display_size)

while True:
    os.exitpoint()
    clock.tick()
    tick_now = time.ticks_ms()
    img = pl.get_frame()
    res = yolo.run(img)

    fps = clock.fps()
    _fps_filter.pop(0)
    _fps_filter.append(fps)
    fps_avg = sum(_fps_filter) / len(_fps_filter)

    candidates = []
    if isinstance(res, list) and len(res) >= 3:
        bboxes = res[0]
        confs = res[2] if len(res) > 2 and res[2] else []
        if isinstance(bboxes, list):
            for i, b in enumerate(bboxes):
                if len(b) >= 4:
                    x, y, w, h = b[0], b[1], b[2], b[3]
                    cx = x + w / 2.0
                    cy = y + h / 2.0
                    cf = confs[i] if i < len(confs) else 1.0
                    candidates.append((cx, cy, w, h, cf))

    best = _filter_ball(candidates)
    if len(candidates) > 0:
        print("raw:{} filt:{} pos:{:+.1f}cm valid:{}".format(
            len(candidates), 1 if best else 0, _pos_cm, _valid))
    yolo.draw_result(res, pl.osd_img)

    if best is not None:
        cx, cy, bw, bh, cf = best
        _last_ball_cx = cx
        _last_ball_cy = cy
        _last_conf = cf

        raw_cm = _px_to_cm(cx)
        k = _get_calib_k(raw_cm)
        raw_cm *= k
        _pos_raw_cm = raw_cm

        _limit_triggered = abs(raw_cm) > LIMIT_CM
        raw_cm = max(-LIMIT_CM, min(LIMIT_CM, raw_cm)) if _limit_triggered else raw_cm

        if _valid:
            _pos_cm = ALPHA * raw_cm + (1.0 - ALPHA) * _pos_cm
        else:
            _pos_cm = raw_cm

        dt = time.ticks_diff(tick_now, _prev_time_ms) / 1000.0
        if dt > 0.001 and _valid:
            _velocity_cms = (_pos_cm - _prev_pos_cm) / dt
        _prev_pos_cm = _pos_cm
        _prev_time_ms = tick_now

        _valid = True
        _new_data = True
        _lost_count = 0

        bbox_x = int(cx - bw / 2)
        bbox_y = int(cy - bh / 2)
        pl.osd_img.draw_string_advanced(bbox_x, bbox_y - 18, 18,
            "conf:{:.2f}".format(cf), color=(255, 255, 0))

    else:
        _lost_count += 1
        if _lost_count <= LOST_HOLD_FRAMES:
            _new_data = False
        else:
            _valid = False
            _new_data = False
            _velocity_cms = 0.0

    if _valid:
        offset_mm = int(_pos_cm * 10)
        offset_mm = max(-120, min(120, offset_mm))
        uart.write("$B,{}\n".format(offset_mm))
    else:
        uart.write("$B,999\n")

    W, H = rgb888p_size[0], rgb888p_size[1]

    # 中心竖线 (以 display_size 为准)
    dx = display_size[0] // 2
    pl.osd_img.draw_line(dx, 0, dx, display_size[1], color=(255,0,0), thickness=1)

    fps_s = "FPS:{}".format(int(fps_avg))
    pl.osd_img.draw_rectangle(W - 90, 2, 88, 26, color=(0, 0, 0), fill=True)
    pl.osd_img.draw_string_advanced(W - 88, 5, 22, fps_s, color=(0, 255, 0))

    if _valid:
        pos_s = "X:{:.0f} pos:{:+.1f}cm".format(_last_ball_cx, _pos_cm)
    else:
        pos_s = "pos: --"
    pl.osd_img.draw_rectangle(2, H - 32, 240, 30, color=(0, 0, 0), fill=True)
    pl.osd_img.draw_string_advanced(5, H - 28, 20, pos_s, color=(0, 255, 0))

    pl.show_image()
    gc.collect()

yolo.deinit()
pl.destroy()
