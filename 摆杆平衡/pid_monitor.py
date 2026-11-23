#!/usr/bin/env python3
"""
摆杆平衡 PID 数据采集 & 自动分析
=================================
从 COM14 读 S3 串口调试数据，采集小球位置时间序列，
分析阶跃响应，计算最优 PID 参数。

用法:
  python pid_monitor.py            # 只采集+显示
  python pid_monitor.py --tune     # 采集+自动调参 (需要WiFi)

依赖: pyserial
"""

import serial
import time
import json
import csv
import sys
import os
from datetime import datetime
from collections import deque

# ---- 配置 ----
SERIAL_PORT = 'COM14'
SERIAL_BAUD = 115200
LOG_FILE = os.path.join(os.path.dirname(__file__), 'pid_log.csv')
SESSION_FILE = os.path.join(os.path.dirname(__file__), 'pid_session.json')

class PIDMonitor:
    def __init__(self, port=SERIAL_PORT, baud=SERIAL_BAUD):
        self.port = port
        self.baud = baud
        self.ser = None
        self.data = []          # [{ts, bp, ok, us, out}]
        self.start_time = None

        # 阶跃检测
        self.last_sp = 0.0
        self.step_start_idx = None
        self.steps = []          # 检测到的阶跃响应

        # PID 分析结果
        self.metrics = {}

    def connect(self):
        """连接串口，处理冲突重试"""
        for attempt in range(5):
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
                time.sleep(0.3)
                self.ser.reset_input_buffer()
                print(f"[OK] 已连接 {self.port}")
                return True
            except serial.SerialException as e:
                print(f"[重试 {attempt+1}/5] 端口被占用: {e}")
                time.sleep(2)
        print("[FAIL] 无法连接串口，请确认没有其他程序占用")
        return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[OK] 串口已关闭")

    def read_line(self):
        """非阻塞读一行"""
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', 'ignore').strip()
                return line
        except Exception as e:
            print(f"[WARN] 读串口错误: {e}")
        return None

    def parse_s3_line(self, line):
        """解析 [S3] 调试行"""
        if not line.startswith('[S3]'):
            return None

        try:
            parts = line[4:].strip().split()
            entry = {}
            for p in parts:
                if '=' in p:
                    k, v = p.split('=', 1)
                    try:
                        entry[k] = float(v)
                    except ValueError:
                        entry[k] = v
            return entry
        except Exception:
            return None

    def run(self, duration_sec=0, progress_callback=None):
        """
        持续采集数据
        duration_sec: 0=无限采集
        progress_callback: 可选回调，每收到数据点调用
        """
        print(f"\n{'='*60}")
        print(f"摆杆平衡 PID 数据采集")
        print(f"{'='*60}")
        print(f"串口: {self.port}@{self.baud}")
        print(f"日志: {LOG_FILE}")
        print(f"按 Ctrl+C 停止采集")
        print(f"{'='*60}\n")

        if not self.connect():
            return

        self.start_time = time.time()
        csv_file = open(LOG_FILE, 'w', newline='', encoding='utf-8')
        csv_w = csv.writer(csv_file)
        csv_w.writerow(['elapsed_ms', 'timestamp', 'bp_cm', 'ball_ok', 'servo_us', 'pid_out'])

        print(f"{'Time':>8s} {'Ball(cm)':>8s} {'OK':>3s} {'Servo':>6s} {'PIDout':>7s}")
        print("-" * 40)

        last_print = 0
        line_buf = ""

        try:
            while True:
                # 检查运行时长
                if duration_sec > 0 and (time.time() - self.start_time) > duration_sec:
                    print("\n[完成] 采集时间到")
                    break

                # 读串口
                try:
                    if self.ser.in_waiting > 0:
                        raw = self.ser.read(self.ser.in_waiting).decode('utf-8', 'ignore')
                        line_buf += raw
                        while '\n' in line_buf:
                            line, line_buf = line_buf.split('\n', 1)
                            entry = self.parse_s3_line(line.strip())
                            if entry and 'bp' in entry:
                                ts = time.time() - self.start_time
                                dp = {
                                    'ts': ts,
                                    'bp': entry.get('bp', 0),
                                    'ok': int(entry.get('ok', 0)),
                                    'us': int(entry.get('us', 0)),
                                    'out': float(entry.get('out', 0)),
                                }
                                self.data.append(dp)

                                # CSV
                                csv_w.writerow([
                                    f"{ts*1000:.0f}",
                                    datetime.now().isoformat(timespec='seconds'),
                                    dp['bp'], dp['ok'], dp['us'], dp['out']
                                ])

                                # 实时显示
                                if ts - last_print > 0.5:
                                    ball_str = f"{dp['bp']:.1f}" if dp['ok'] else "---"
                                    ok_str = "✓" if dp['ok'] else "✗"
                                    print(f"{ts:8.1f}s {ball_str:>8s} {ok_str:>3s} {dp['us']:>6d} {dp['out']:>+7.1f}")
                                    last_print = ts

                                # 回调
                                if progress_callback:
                                    progress_callback(dp, self.data)
                except serial.SerialException:
                    time.sleep(0.1)
                    continue

                time.sleep(0.01)  # 不让 CPU 空转

        except KeyboardInterrupt:
            print("\n\n[中断] 用户停止采集")

        finally:
            csv_file.close()
            self.disconnect()
            print(f"\n[OK] 共采集 {len(self.data)} 个数据点")
            print(f"[OK] 日志已保存: {LOG_FILE}")

    def analyze(self):
        """分析采集到的数据，计算 PID 性能指标"""
        if len(self.data) < 10:
            print("[WARN] 数据太少，无法分析")
            return None

        # 只取有效球数据
        valid = [d for d in self.data if d['ok'] == 1]
        if len(valid) < 5:
            print("[WARN] 有效球数据太少")
            return None

        bp_vals = [d['bp'] for d in valid]
        ts_vals = [d['ts'] for d in valid]
        out_vals = [d['out'] for d in valid]

        # 基本统计
        bp_mean = sum(bp_vals) / len(bp_vals)
        bp_abs = [abs(b) for b in bp_vals]
        bp_mae = sum(bp_abs) / len(bp_abs)  # 平均绝对偏差

        # 稳态分析 (取最后 20% 数据作为稳态)
        n_steady = max(5, len(valid) // 5)
        steady_bp = bp_vals[-n_steady:]
        steady_mean = sum(steady_bp) / len(steady_bp)
        steady_std = (sum((b - steady_mean)**2 for b in steady_bp) / len(steady_bp)) ** 0.5

        # 振荡检测: 符号变化次数
        oscillations = 0
        for i in range(1, len(bp_vals)):
            if bp_vals[i] * bp_vals[i-1] < 0:  # 过零
                oscillations += 1

        # PID 输出震荡
        out_osc = 0
        for i in range(1, len(out_vals)):
            if out_vals[i] * out_vals[i-1] < 0:
                out_osc += 1

        self.metrics = {
            'total_points': len(self.data),
            'valid_points': len(valid),
            'valid_pct': len(valid) / len(self.data) * 100,
            'bp_mean': bp_mean,
            'bp_mae': bp_mae,
            'steady_mean': steady_mean,
            'steady_std': steady_std,
            'oscillations': oscillations,
            'duration': ts_vals[-1] - ts_vals[0],
            'out_oscillations': out_osc,
            'bp_min': min(bp_vals),
            'bp_max': max(bp_vals),
            'out_range': max(abs(min(out_vals)), abs(max(out_vals))),
        }

        return self.metrics

    def detect_step_response(self, sp_history=None):
        """
        检测阶跃响应（设定点变化）
        sp_history: 如果通过 WiFi API 获取了 setpoint 历史，直接传入
        """
        if len(self.data) < 20:
            return []

        valid = [d for d in self.data if d['ok'] == 1]
        if len(valid) < 10:
            return []

        # 简化的阶跃检测: 球位置快速变化 + 收敛
        # 寻找: bp 从接近 0 快速偏离 → 震荡 → 稳定
        steps = []
        bp_vals = [d['bp'] for d in valid]
        ts_vals = [d['ts'] for d in valid]

        # 检测大幅偏离
        threshold = 1.0  # cm, 偏离超过此值视为阶跃响应
        i = 0
        while i < len(bp_vals):
            if abs(bp_vals[i]) > threshold:
                # 找到一个阶跃
                step_start = i
                # 找峰值
                peak_idx = step_start
                peak_val = abs(bp_vals[step_start])
                j = step_start + 1
                while j < len(bp_vals) and j - step_start < 100:  # 最多100个采样点
                    if abs(bp_vals[j]) > peak_val:
                        peak_val = abs(bp_vals[j])
                        peak_idx = j
                    # 检查是否已经稳定
                    if j - step_start > 10:
                        recent = bp_vals[max(j-5, step_start):j+1]
                        if all(abs(b) < threshold for b in recent):
                            break
                    j += 1

                if peak_idx > step_start:
                    steps.append({
                        'start_idx': step_start,
                        'start_ts': ts_vals[step_start],
                        'peak_idx': peak_idx,
                        'peak_val': bp_vals[peak_idx],
                        'overshoot_pct': abs(bp_vals[peak_idx]) / abs(bp_vals[step_start]) * 100 if bp_vals[step_start] != 0 else 0,
                    })
                    i = j
                else:
                    i += 1
            else:
                i += 1

        self.steps = steps
        return steps

    def suggest_pid(self, current_kp=None, current_kd=None, current_ki=None):
        """
        基于采集数据建议 PID 参数

        启发式规则:
        - 持续震荡 → KD 不足 (加大 KD 或减小 KP)
        - 稳态误差大 → KI 不足
        - 超调过大 → KD 不足 或 KP 过大
        - 收敛太慢 → KP 不足
        - PID 输出频繁切换方向 → KD 不足 (噪声放大)
        """
        if not self.metrics:
            self.analyze()
        if not self.metrics:
            return {'kp': 40, 'kd': 18, 'ki': 0.5, 'reason': '默认参数 (无足够数据)'}

        m = self.metrics

        # 默认起点
        kp = 40.0 if current_kp is None else current_kp
        kd = 18.0 if current_kd is None else current_kd
        ki = 0.5 if current_ki is None else current_ki

        reasons = []

        # 1. 震荡分析 — 最重要
        # 每个有效数据点大约代表 1 秒 (1Hz 串口输出)
        # 如果每秒都有过零，说明严重震荡
        osc_rate = m['oscillations'] / max(m['valid_points'], 1)

        if osc_rate > 0.3:  # 超过 30% 的采样点过零 → 严重震荡
            # 震荡太厉害，减小 KP 或增大 KD
            if kd / max(kp, 0.1) < 0.4:
                kd *= 1.5  # KD 比例太低，加大 KD
                reasons.append(f"严重震荡(过零率{osc_rate:.0%})→KD↑×1.5")
            else:
                kp *= 0.7  # KD 已经不低，减小 KP
                reasons.append(f"严重震荡(过零率{osc_rate:.0%})→KP↓×0.7")
        elif osc_rate > 0.15:  # 中等震荡
            kp *= 0.85
            kd *= 1.2
            reasons.append(f"中等震荡(过零率{osc_rate:.0%})→KP↓×0.85, KD↑×1.2")

        # 2. 稳态误差
        steady_err = abs(m['steady_mean'])
        if steady_err > 0.5:  # 稳态偏差 > 0.5cm
            if ki < 0.1:
                ki = 0.3
                reasons.append(f"稳态偏差({steady_err:.1f}cm)→KI 增加到 0.3")
            else:
                ki *= 1.3
                reasons.append(f"稳态偏差({steady_err:.1f}cm)→KI↑×1.3")
        elif steady_err < 0.2 and ki > 0.1:
            # 稳态很好，可以尝试减小 KI 避免积分饱和
            pass

        # 3. PID 输出频繁切换 → KD 噪声放大
        out_osc_rate = m['out_oscillations'] / max(m['valid_points'], 1)
        if out_osc_rate > 0.5 and kd > 5:
            kd *= 0.8
            reasons.append(f"PID输出高频切换→KD↓×0.8(降噪)")

        # 4. 超调分析 (从阶跃数据)
        if self.steps:
            max_overshoot = max(s.get('overshoot_pct', 0) for s in self.steps)
            if max_overshoot > 50:
                kp *= 0.8
                kd *= 1.3
                reasons.append(f"大超调({max_overshoot:.0f}%)→KP↓×0.8, KD↑×1.3")

        # 5. 收敛速度 (从持续时间看)
        # 如果花了很长时间才稳定
        if m['duration'] > 10 and osc_rate < 0.1:
            # 稳定但慢，可以稍微加大 KP
            kp *= 1.1
            reasons.append(f"收敛偏慢→KP↑×1.1")

        # 限幅
        kp = max(1, min(100, kp))
        kd = max(0.5, min(50, kd))
        ki = max(0, min(2.0, ki))

        return {
            'kp': round(kp, 1),
            'kd': round(kd, 1),
            'ki': round(ki, 2),
            'reason': '; '.join(reasons) if reasons else '参数合理，无需调整'
        }

    def print_report(self):
        """打印分析报告"""
        if not self.metrics:
            self.analyze()

        m = self.metrics
        if not m:
            print("\n[无数据] 无法生成报告")
            return

        print(f"\n{'='*60}")
        print(f"PID 性能分析报告")
        print(f"{'='*60}")
        print(f"数据点数: {m['total_points']} (有效球: {m['valid_points']}, {m['valid_pct']:.0f}%)")
        print(f"采集时长: {m['duration']:.1f}s")
        print(f"球位置均值: {m['bp_mean']:.2f} cm")
        print(f"平均绝对偏差: {m['bp_mae']:.2f} cm")
        print(f"稳态均值: {m['steady_mean']:.2f} cm")
        print(f"稳态标准差: {m['steady_std']:.2f} cm")
        print(f"过零次数: {m['oscillations']}")
        print(f"PID输出过零: {m['out_oscillations']}")
        print(f"球位置范围: [{m['bp_min']:.1f}, {m['bp_max']:.1f}] cm")
        print(f"PID输出范围: ±{m['out_range']:.1f}")

        # 判定
        print(f"\n{'─'*40}")
        if m['steady_std'] < 0.3 and m['bp_mae'] < 0.5:
            print("✅ 性能良好! 球稳定在中心")
        elif m['steady_std'] < 0.8:
            print("🟡 可接受，有小幅波动")
        elif m['oscillations'] > m['valid_points'] * 0.3:
            print("🔴 严重震荡! 需要调整 KD 或减小 KP")
        elif m['bp_mae'] > 1.0:
            print("🟠 偏差较大，需要增大 KP 或检查 KI")
        else:
            print("🟡 一般，可以进一步优化")

        if self.steps:
            print(f"\n阶跃响应 ({len(self.steps)} 个):")
            for i, s in enumerate(self.steps):
                print(f"  #{i}: 峰值={s['peak_val']:.1f}cm 超调={s['overshoot_pct']:.0f}%")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='摆杆平衡 PID 数据采集 & 分析')
    parser.add_argument('--port', default='COM14', help='串口号')
    parser.add_argument('--duration', type=int, default=0, help='采集时长(秒), 0=无限')
    parser.add_argument('--analyze', action='store_true', help='只分析已有日志')
    args = parser.parse_args()

    monitor = PIDMonitor(port=args.port)

    if args.analyze:
        # 从 CSV 加载已有数据
        if os.path.exists(LOG_FILE):
            with open(LOG_FILE, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    monitor.data.append({
                        'ts': float(row['elapsed_ms']) / 1000,
                        'bp': float(row['bp_cm']),
                        'ok': int(row['ball_ok']),
                        'us': int(row['servo_us']),
                        'out': float(row['pid_out']),
                    })
            print(f"[OK] 从日志加载 {len(monitor.data)} 个数据点")
            monitor.analyze()
            monitor.detect_step_response()
            monitor.print_report()
            sug = monitor.suggest_pid()
            print(f"\n建议 PID: KP={sug['kp']} KD={sug['kd']} KI={sug['ki']}")
            print(f"原因: {sug['reason']}")
        else:
            print(f"[WARN] 日志文件不存在: {LOG_FILE}")
    else:
        monitor.run(duration_sec=args.duration)
        monitor.analyze()
        monitor.detect_step_response()
        monitor.print_report()
        sug = monitor.suggest_pid()
        print(f"\n建议 PID: KP={sug['kp']} KD={sug['kd']} KI={sug['ki']}")
        print(f"原因: {sug['reason']}")


if __name__ == '__main__':
    main()
