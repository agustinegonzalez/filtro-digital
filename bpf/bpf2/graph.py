import serial
import struct
import numpy as np
import pyqtgraph as pg
import signal
import sys
from pyqtgraph.Qt import QtCore, QtWidgets

MAGIC = b"\xA5\x5A\xA5\x5A"

ser = serial.Serial(
    "/dev/ttyACM0",
    115200,
    timeout=0
)

app = QtWidgets.QApplication([])

win = pg.GraphicsLayoutWidget(title="ADC + Filtro IIR en tiempo real")
p1 = win.addPlot(title="Entrada (VIN)")
curve_in = p1.plot(pen='y')

win.nextRow()
p2 = win.addPlot(title="Salida filtrada (VOUT)")
curve_out = p2.plot(pen='c')

# Ventana de FFT
win2 = pg.GraphicsLayoutWidget(title="Espectro")
pfft = win2.addPlot(title="FFT de salida")
curve_fft = pfft.plot(pen='m')

win.show()
win2.show()

buffer = bytearray()
fs_actual = 200000   # valor por defecto en caso de no recibir todavía


def leer_trama():
    global buffer, fs_actual

    data = ser.read(ser.in_waiting or 1)
    if data:
        buffer.extend(data)

    # Evitar buffer infinito
    if len(buffer) > 200000:
        buffer = buffer[-50000:]

    idx = buffer.find(MAGIC)
    if idx == -1:
        return None, None

    header_size = 4 + 4 + 2 + 2
    if len(buffer) < idx + header_size:
        return None, None

    p = idx + 4
    fs = struct.unpack("<I", buffer[p:p+4])[0]; p += 4
    n  = struct.unpack("<H", buffer[p:p+2])[0]; p += 2
    seq = struct.unpack("<H", buffer[p:p+2])[0]; p += 2

    fs_actual = fs

    payload_size = n * 4
    total_size = header_size + payload_size + 4

    if len(buffer) < idx + total_size:
        return None, None

    frame = buffer[idx : idx + total_size]
    del buffer[: idx + total_size]

    payload = frame[12:-4]

    data = np.frombuffer(payload, dtype=np.uint16).reshape(-1, 2)
    vin = data[:, 0]
    vout = data[:, 1]

    return vin, vout


def update():
    global fs_actual
    vin, vout = leer_trama()
    if vin is None:
        return

    MAXPLOT = 2000
    curve_in.setData(vin[-MAXPLOT:])
    curve_out.setData(vout[-MAXPLOT:])

    # FFT con ventana Hann
    w = np.hanning(len(vout))
    fft = np.abs(np.fft.rfft((vout - np.mean(vout)) * w))
    freqs = np.fft.rfftfreq(len(vout), 1/fs_actual)

    curve_fft.setData(freqs, fft)


timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10)


def salir(*args):
    print("Cerrando...")
    ser.close()
    app.quit()
    sys.exit()


signal.signal(signal.SIGINT, salir)
signal.signal(signal.SIGTERM, salir)

app.exec()

