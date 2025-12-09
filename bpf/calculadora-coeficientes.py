import numpy as np
from scipy import signal

fs = 300000
fc = 44000
bw = 7000

f1 = (fc - bw/2) / (fs/2)
f2 = (fc + bw/2) / (fs/2)

b, a = signal.cheby1(
    N=1,       
    rp=3,             
    Wn=[f1, f2],     
    btype='bandpass'
)

b = b * 8   # ganancia v/v 8
print(b)
print(a)

