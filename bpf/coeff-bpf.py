import numpy as np
from scipy import signal

fs = 200000
fc = 44000
bw = 7000

f1 = (fc - bw/2) / (fs/2)
f2 = (fc + bw/2) / (fs/2)

b, a = signal.cheby1(
    N=1,              # ← orden 1 → Biquad
    rp=3,             # ripple 3 dB
    Wn=[f1, f2],      # banda
    btype='bandpass'
)

b = b * 8   # ganancia v/v 8
print(b)
print(a)

