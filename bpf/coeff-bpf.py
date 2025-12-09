import numpy as np
from scipy import signal
from scipy.signal import iirpeak

fs = 200000       # Hz
Fc = 44000        # Hz
BW = 7000         # Hz

fp1 = Fc - BW/2   # 40500 Hz
fp2 = Fc + BW/2   # 47500 Hz
order = 1
Wn = [fp1/(fs/2), fp2/(fs/2)]

# Orden 2 = un solo biquad
b, a = signal.butter(order, Wn, btype='bandpass')

print("b =", b)
print("a =", a)

f0 = 44000

Q = f0 / BW  # Q ≈ 6.28

b, a = iirpeak(f0, Q, fs=fs)
print("botro =", b)
print("aotro =", a)

