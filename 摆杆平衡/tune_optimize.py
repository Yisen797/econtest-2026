"""优化: 归位参数 + PID"""
import serial, time

ser = serial.Serial('COM14', 115200, timeout=0.2)
time.sleep(1.0); ser.reset_input_buffer()

def cmd(s):
    ser.write((s + '\n').encode()); ser.flush()
    time.sleep(0.03)

# Current best PID
KP, KD, KI = 0.8, 60, 2.0

# Test: vary kick + threshold manually via $SERVO + monitoring
# Can't easily change SEQ_HOME_R params via serial, so just test PID more

# Wider PID grid focused on low KP
grid = [
    (0.8, 60, 2.0),
    (0.6, 55, 2.5),
    (0.5, 50, 3.0),
    (0.8, 65, 1.8),
    (0.7, 60, 2.2),
    (0.9, 55, 1.8),
    (0.5, 60, 2.5),
    (0.7, 70, 2.0),
]

def run_one(kp, kd, ki):
    cmd(f'$PID,{kp},{kd},{ki:.2f}')
    cmd('$HOME,1500')
    time.sleep(2)
    ser.reset_input_buffer()
    cmd('$SEQ')
    time.sleep(8)
    buf = ser.read(ser.in_waiting).decode('utf-8','ignore')
    lines = buf.split('\n')
    done = any('DONE' in l for l in lines)
    bps = []
    for l in lines:
        if '[S3]' in l and 'ok=1' in l:
            for p in l.split():
                if p.startswith('bp='): bps.append(float(p.split('=')[1]))
    if len(bps) < 5:
        return done, 99, 99, len(bps)
    # Check if ball stabilizes near 0 at the end
    end_bps = bps[-15:] if len(bps)>=15 else bps[-5:]
    avg = sum(end_bps)/len(end_bps)
    amp = (max(bps)-min(bps))/2
    return done, avg, amp, len(bps)

def reset():
    cmd('$PID,0,0,0')
    cmd('$SERVO,1500')
    time.sleep(2)

print(f'{"KP":>5s} {"KD":>5s} {"KI":>5s} {"done":>5s} {"steady":>7s} {"amp":>6s} {"n":>4s} score')
print('-' * 50)
best_score = 999
best = None

for kp, kd, ki in grid:
    done, avg, amp, n = run_one(kp, kd, ki)
    score = abs(avg)*10 + amp*3 + (0 if done else 50)
    print(f'{kp:5.1f} {kd:5.0f} {ki:5.2f} {str(done):>5s} {avg:7.1f} {amp:6.1f} {n:4d}  {score:5.0f}')
    if score < best_score:
        best_score = score
        best = (kp, kd, ki, avg, amp)
    reset()

print(f'\n最佳: KP={best[0]:.1f} KD={best[1]:.0f} KI={best[2]:.2f} steady={best[3]:.1f} amp={best[4]:.1f}')
ser.close()
