# 2026电赛项目 — 第三题已解决

## 第三题 摆杆平衡 ✅

| 项目 | 值 |
|------|-----|
| PID | **KP=40 KD=18 KI=0.5** |
| Home | 1500 (NVS) |
| 步进 | 20us平滑 |
| 归零键 | **GPIO47** (INPUT_PULLUP, 按GND) |
| 第三题键 | **GPIO38** |
| 归零流程 | 推左800us→右倾1550us等球到-4→刹车1350us→PID稳零→锁止 |
| 第三题流程 | 0→+4(3s)→减速1s→-5(6s)→停留 |
| 稳态 | 中心<1cm, -5约2cm |

## 第一题 图传 ✅
OpenMV硬件JPEG→S3→Web

## 第二题 寻迹小车 🟡
~22s, 需≤20s

## 编译烧录

```bash
cd "<项目根目录>/摆杆平衡"
~/.platformio/penv/Scripts/pio run -t upload --upload-port COM14
```

## 关键踩坑
- OpenMV ps_malloc崩溃 → 已注释, 加null guard
- PID D-heavy不如P-heavy → KP=40 KD=18最优
- 步进限幅不能可变 → 固定20us
- 锁止需重置 → pid_slew_reset时清locked
- GPIO2偶尔失效 → 改GPIO47
- -5冲过头 → SEQ_TO_N5限幅1250us
- 直接跳-5过冲 → 加1s减速过渡
