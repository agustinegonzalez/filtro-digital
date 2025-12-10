from scipy import signal
import matplotlib.pyplot as plt
import numpy as np
#cheby1(N, rp, Wn, btype='low', analog=False, output='ba', fs=None)
fs = 300000
fc = 44000
bw = 7000
#f1 = (fc - bw/2) / (fs/2)
#f2 = (fc + bw/2) / (fs/2)
f1=40.5e3
f2=47.5e3
b, a = signal.cheby1(1, 3,[f1,f2] , 'bandpass', analog=True)
w, h = signal.freqs(b*8, a)
#f=w/(2*np.pi)
plt.semilogx(w, 20 * np.log10(abs(h)))
plt.title('Chebyshev respuesta en frecuencia (Amax=3db)')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Amplitude [dB]')
plt.margins(0, 0.1)
plt.grid(which='both', axis='both')
plt.axvline(100, color='green') # cutoff frequency
plt.axhline(-5, color='green') # rp
plt.xlim([1000,1000e3])
plt.savefig("bode.png")
plt.show()
