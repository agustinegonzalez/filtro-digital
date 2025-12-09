import serial
import struct
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore

MAGIC = b"\xA5\x5A\xA5\x5A"

ser = serial.Serial("/dev/ttyS0", 115200, timeout=1)

app = QtGui.QApplication([])
win = pg.GraphicsWindow(title="ADC + Filtro IIR en tiempo real")
p1 = win.addPlot(title="Entrada (VIN)")
curve_in = p1.plot(pen='y')
win.nextRow()
p2 = win.addPlot(title="Salida filtrada (VOUT)")
curve_out = p2.plot(pen='c')
#FFT
win2 = pg.GraphicsWindow(title="Espectro")
pfft = win2.addPlot(title="FFT de salida")
curve_fft = pfft.plot(pen='m')

def leer_trama():
    if ser.read(4) != MAGIC:
        return None, None

    fs = struct.unpack("<I", ser.read(4))[0]
    n = struct.unpack("<H", ser.read(2))[0]
    _seq = struct.unpack("<H", ser.read(2))[0]

    frame_bytes = ser.read(n * 4)
    ser.read(4)  # MAGIC final

    data = np.frombuffer(frame_bytes, dtype=np.uint16)
    data = data.reshape(-1, 2)

    vin = data[:, 0]
    vout = data[:, 1]

    return vin, vout

def update():
    vin, vout = leer_trama()
    if vin is None:
        return
    curve_in.setData(vin)
    curve_out.setData(vout)
    # FFT rápida de vout
    fft = np.abs(np.fft.rfft(vout - np.mean(vout)))
    curve_fft.setData(fft)

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(1)

QtGui.QApplication.instance().exec_()

