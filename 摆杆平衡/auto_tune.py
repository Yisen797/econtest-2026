#!/usr/bin/env python3
"""
摆杆平衡 自动PID调参
====================
策略: 临界比例度法 + 阶跃响应分析
1. 摆舵机找球
2. KI=KD=0, 增大KP直到等幅振荡 → 临界增益 Ku, 临界周期 Tu
3. Ziegler-Nichols: KP=0.6*Ku, KD=0.075*Ku*Tu, KI=1.2*Ku/Tu
4. 阶跃测试微调
"""

import serial
import time
import sys
import json
from collections import deque

PORT = 'COM14'
BAUD = 115200
HOME_US = 1265  # 默认原点

class AutoTuner:
    def __init__(self):
        self.ser = None
        self.data = []
        self.t0 = None

    def connect(self):
        for attempt in range(5):
            try:
                self.ser = serial.Serial(PORT, BAUD, timeout=0.2)
                time.sleep(1.0)
                self.ser.reset_input_buffer()
                print(f"[OK] 已连接 {PORT}")
                return True
            except Exception as e:
                print(f"[重试 {attempt+1}/5] {e}")
                time.sleep(2)
        return False

    def cmd(self, s):
        """发送串口命令"""
        self.ser.write((s + '\n').encode())
        self.ser.flush()
        time.sleep(0.05)

    def set_pid(self, kp, kd, ki):
        self.cmd(f'$PID,{kp:.1f},{kd:.1f},{ki:.2f}')
        print(f'  -> PID: KP={kp:.1f} KD={kd:.1f} KI={ki:.2f}')

    def set_sp(self, sp):
        self.cmd(f'$SP,{sp:.1f}')

    def set_servo(self, us):
        self.cmd(f'$SERVO,{us}')

    def sweep_find_ball(self):
        """左右摆动舵机，让小球滚入K230视野"""
        print("\n=== 摆动找球 ===")
        # 先归中
        self.set_servo(HOME_US)
        time.sleep(1)

        # 左右扫描: 从中心向两端摆动，幅度逐渐加大
        sweeps = [
            (HOME_US+100, HOME_US-100, 0.5),
            (HOME_US+200, HOME_US-200, 0.8),
            (HOME_US+300, HOME_US-300, 1.0),
            (HOME_US+400, HOME_US-400, 1.2),
            (HOME_US+500, HOME_US-500, 1.5),
        ]

        for right, left, dur in sweeps:
            if right > 2500: right = 2500
            if left < 500: left = 500

            # 向右倾斜
            self.set_servo(right)
            time.sleep(dur)
            if self._check_ball(2):
                self.set_sp(0)
                return True

            # 向左倾斜
            self.set_servo(left)
            time.sleep(dur)
            if self._check_ball(2):
                self.set_sp(0)
                return True

        # 如果还没找到，大范围扫
        print("  大范围扫描...")
        for us in range(HOME_US-400, HOME_US+401, 100):
            if us < 500 or us > 2500: continue
            self.set_servo(us)
            time.sleep(0.3)
            if self._check_ball(1):
                self.set_sp(0)
                return True

        print("[FAIL] 未找到小球!")
        return False

    def _check_ball(self, timeout=2):
        """检查是否有小球检测"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.ser.in_waiting:
                buf = self.ser.read(self.ser.in_waiting).decode('utf-8','ignore')
                for l in buf.split('\n'):
                    if '[S3]' in l and 'ok=1' in l:
                        parts = l.strip().split()
                        for p in parts:
                            if p.startswith('bp='):
                                bp = float(p.split('=')[1])
                                print(f'  检测到小球: bp={bp:.1f}cm')
                                return True
            time.sleep(0.03)
        return False

    def collect(self, duration, label=""):
        """采集一段数据，返回有效球数据点列表"""
        raw = []
        t0 = time.time()
        line_buf = ''

        while time.time() - t0 < duration:
            if self.ser.in_waiting:
                buf = self.ser.read(self.ser.in_waiting).decode('utf-8','ignore')
                line_buf += buf
                while '\n' in line_buf:
                    line, line_buf = line_buf.split('\n', 1)
                    if '[S3]' in line:
                        entry = {'ts': time.time() - t0}
                        for p in line.strip().split():
                            if '=' in p:
                                k, v = p.split('=', 1)
                                try:
                                    entry[k] = float(v)
                                except ValueError:
                                    entry[k] = v
                        if 'bp' in entry:
                            raw.append(entry)
            time.sleep(0.005)

        valid = [d for d in raw if d.get('ok', 0) == 1]
        print(f"  [{label}] 采集 {len(raw)}点, 有效球 {len(valid)}点 ({duration}s)")
        return valid

    def analyze_oscillation(self, data):
        """分析振荡特性: 返回 (振幅, 周期, 过零率)"""
        if len(data) < 10:
            return 0, 0, 0

        bps = [d['bp'] for d in data]
        amplitude = (max(bps) - min(bps)) / 2

        # 检测过零周期
        zero_crossings = []
        for i in range(1, len(bps)):
            if bps[i] * bps[i-1] <= 0:
                zero_crossings.append(data[i]['ts'])

        if len(zero_crossings) >= 3:
            periods = [zero_crossings[i+2] - zero_crossings[i] for i in range(len(zero_crossings)-2)]
            period = sum(periods) / len(periods) if periods else 0
        else:
            period = 0

        osc_rate = len(zero_crossings) / max(len(bps), 1)
        return amplitude, period, osc_rate

    def find_critical_gain(self):
        """
        KI=0, KD=0, 逐步增大KP直到等幅振荡
        返回 (Ku=临界增益, Tu=临界周期)
        """
        print("\n=== 临界增益搜索 ===")
        self.set_pid(5, 0, 0)
        self.set_sp(0)
        time.sleep(1)

        # 小扰动: 偏离设点看响应
        self.set_sp(2)
        time.sleep(0.5)
        self.set_sp(0)
        time.sleep(1)

        kp = 5
        while kp <= 80:
            self.set_pid(kp, 0, 0)
            time.sleep(1)

            # 给一个扰动
            self.set_sp(3)
            time.sleep(0.5)
            self.set_sp(0)

            data = self.collect(4, f"KP={kp}")
            if len(data) < 20:
                print(f"  KP={kp}: 球丢失! 回退")
                kp -= 10
                break

            amp, period, osc_rate = self.analyze_oscillation(data)

            # 判断: 连续过零(osc_rate>0.3) + 振幅不衰减 → 等幅振荡
            if osc_rate > 0.3 and amp > 0.5:
                # 检查是否等幅 (后一半振幅 ≈ 前一半)
                n = len(data)
                bp1 = [d['bp'] for d in data[:n//2]]
                bp2 = [d['bp'] for d in data[n//2:]]
                amp1 = (max(bp1)-min(bp1))/2 if bp1 else 0
                amp2 = (max(bp2)-min(bp2))/2 if bp2 else 0
                ratio = amp2 / max(amp1, 0.01)

                print(f"  KP={kp}: amp={amp:.1f}cm T={period:.1f}s osc={osc_rate:.0%} amp1/amp2={ratio:.1f}")

                if 0.5 < ratio < 2.0:  # 等幅
                    Ku = kp
                    Tu = period
                    print(f"\n*** 临界增益 Ku={Ku:.0f} Tu={Tu:.1f}s ***")
                    return Ku, Tu

            print(f"  KP={kp}: amp={amp:.1f}cm osc={osc_rate:.0%}")

            if osc_rate > 0.5:  # 严重振荡, 已经过头
                Ku = max(kp - 10, 5)
                Tu = period if period > 0 else 0.5
                print(f"\n*** 振荡过大, 估算 Ku={Ku:.0f} Tu={Tu:.1f}s ***")
                return Ku, Tu

            kp += 10

        # 未找到清晰临界点
        print(f"\n*** 未找到临界振荡, 使用 KP={kp-10} 作为参考 ***")
        return max(kp-10, 10), 0.5

    def zn_tune(self, Ku, Tu):
        """Ziegler-Nichols 调参"""
        # 经典ZN: KP=0.6*Ku, KI=1.2*Ku/Tu, KD=0.075*Ku*Tu
        kp = 0.6 * Ku
        ki = 1.2 * Ku / max(Tu, 0.1)
        kd = 0.075 * Ku * Tu

        # P控制的KI=0
        # 实际中, 50Hz控制周期下需要调整
        # PID_DT=0.02s, 所以KI需要缩放
        # ZN的KI是连续时间的, 离散需要乘DT
        ki = ki * 0.02  # DT scaling

        # 限幅
        kp = max(2, min(80, kp))
        kd = max(0.5, min(40, kd))
        ki = max(0, min(2.0, ki))

        return round(kp, 1), round(kd, 1), round(ki, 2)

    def step_test(self, kp, kd, ki, sp_target=3):
        """单次阶跃测试: 设点跳变sp_target, 分析响应"""
        self.set_pid(kp, kd, ki)
        self.set_sp(0)
        time.sleep(1.5)

        # 清积分
        self.cmd('$PID,{:.1f},{:.1f},{:.2f}'.format(kp, kd, ki))
        time.sleep(0.5)

        # 阶跃
        self.set_sp(sp_target)
        data = self.collect(6, f"STEP sp={sp_target} KP={kp} KD={kd} KI={ki}")
        self.set_sp(0)
        time.sleep(1)

        if len(data) < 15:
            return {'error': '球丢失'}

        bps = [d['bp'] for d in data]
        steady = bps[-10:] if len(bps) >= 10 else bps
        steady_err = abs(sum(steady)/len(steady) - sp_target)
        overshoot = max(0, max(bps) - sp_target) if sp_target > 0 else max(0, abs(min(bps) + sp_target))
        settling = 999
        for i in range(len(bps)):
            if all(abs(b - sp_target) < 0.5 for b in bps[i:]):
                settling = data[i]['ts']
                break

        amp, period, osc_rate = self.analyze_oscillation(data)

        # 综合评分 (越小越好)
        score = steady_err * 10 + overshoot * 5 + osc_rate * 20 + settling * 0.5

        return {
            'steady_err': steady_err,
            'overshoot': overshoot,
            'settling': settling,
            'osc_rate': osc_rate,
            'amplitude': amp,
            'period': period,
            'score': score,
            'n_points': len(data),
        }

    def fine_tune(self, kp0, kd0, ki0):
        """微调: 在ZN参数附近搜索最优"""
        print("\n=== 微调优化 ===")

        candidates = [
            (kp0, kd0, ki0),
            (kp0*0.8, kd0, ki0),
            (kp0*1.2, kd0, ki0),
            (kp0, kd0*0.8, ki0),
            (kp0, kd0*1.2, ki0),
            (kp0, kd0, ki0*0.5),
            (kp0, kd0, ki0*1.5),
            (kp0*0.9, kd0*1.1, ki0),
            (kp0*1.1, kd0*0.9, ki0),
        ]

        best = None
        best_score = 999

        for kp, kd, ki in candidates:
            kp = round(kp, 1)
            kd = round(kd, 1)
            ki = round(ki, 2)
            kp = max(2, min(80, kp))
            kd = max(0.5, min(40, kd))
            ki = max(0, min(2.0, ki))

            r = self.step_test(kp, kd, ki, sp_target=3)
            if 'error' in r:
                print(f"  KP={kp} KD={kd} KI={ki}: {r['error']}")
                continue

            print(f"  KP={kp:.0f} KD={kd:.0f} KI={ki:.2f}: "
                  f"err={r['steady_err']:.1f}cm ovs={r['overshoot']:.1f}cm "
                  f"osc={r['osc_rate']:.0%} stl={r['settling']:.1f}s score={r['score']:.1f}")

            if r['score'] < best_score:
                best_score = r['score']
                best = (kp, kd, ki, r)

        return best

    def run(self):
        print("=" * 60)
        print("摆杆平衡 自动PID调参")
        print("=" * 60)

        if not self.connect():
            return

        # Step 1: 找球
        self.set_pid(0, 0, 0)  # 先关PID
        time.sleep(0.5)

        if not self._check_ball(3):
            print("小球不在视野中，开始摆动找球...")
            if not self.sweep_find_ball():
                print("\n[FAIL] 无法找到小球! 请检查:")
                print("  1. K230是否已上电?")
                print("  2. 小球是否在平台上?")
                print("  3. 手动倾斜平台让球出现")
                return

        # Step 2: 临界增益法
        print("\n小球已就位，开始临界增益搜索...")
        time.sleep(1)

        # 先做一次大方差摆动确保球在视野中
        self.set_servo(HOME_US)
        time.sleep(1)
        self.set_sp(0)

        Ku, Tu = self.find_critical_gain()
        print(f"\n临界参数: Ku={Ku:.0f} Tu={Tu:.1f}s")

        # Step 3: ZN调参
        kp, kd, ki = self.zn_tune(Ku, Tu)
        print(f"\nZN建议: KP={kp:.1f} KD={kd:.1f} KI={ki:.2f}")

        # Step 4: 微调
        best = self.fine_tune(kp, kd, ki)
        if best:
            kp, kd, ki, r = best
            print(f"\n{'='*60}")
            print(f"最优参数: KP={kp:.0f} KD={kd:.0f} KI={ki:.2f}")
            print(f"稳态误差={r['steady_err']:.1f}cm 超调={r['overshoot']:.1f}cm 收敛={r['settling']:.1f}s")
            print(f"{'='*60}")

            # 应用最优参数
            self.set_pid(kp, kd, ki)
            self.set_sp(0)

            # 跑一次自动流程验证
            print("\n=== 验证: 自动流程 O->+5->0->-5->0 ===")
            for sp_target in [5, 0, -5, 0]:
                print(f"\n--- SP={sp_target} ---")
                self.set_sp(sp_target)
                data = self.collect(3, f"VERIFY SP={sp_target}")
                if len(data) > 10:
                    bps = [d['bp'] for d in data]
                    last5 = bps[-5:]
                    err = abs(sum(last5)/len(last5) - sp_target)
                    amp = (max(bps)-min(bps))/2
                    print(f"  终值={sum(last5)/len(last5):.1f}cm 误差={err:.1f}cm 振幅={amp:.1f}cm")
                else:
                    print("  球丢失!")
                time.sleep(0.5)

        self.set_sp(0)
        print("\n=== 调参完成 ===")


if __name__ == '__main__':
    tuner = AutoTuner()
    tuner.run()
