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
    timeout=0   # NO bloqueante
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

def leer_trama():
    global buffer

    # Leer sin bloquear
    data = ser.read(ser.in_waiting or 1)
    if data:
        buffer.extend(data)

    # Buscar inicio de trama
    idx = buffer.find(MAGIC)
    if idx == -1:
        return None, None

    header_size = 4 + 4 + 2 + 2  # MAGIC + fs + N + seq
    if len(buffer) < idx + header_size:
        return None, None

    p = idx + 4
    fs = struct.unpack("<I", buffer[p:p+4])[0]; p += 4
    n = struct.unpack("<H", buffer[p:p+2])[0]; p += 2
    seq = struct.unpack("<H", buffer[p:p+2])[0]; p += 2

    payload_size = n * 4
    total_size = header_size + payload_size + 4  # + MAGIC final

    # verifica si trama llegó completa
    if len(buffer) < idx + total_size:
        return None, None

    frame = buffer[idx : idx + total_size]

    # Borrar del buffer
    del buffer[: idx + total_size]

    # Extraer payload (salteando MAGIC+header y MAGIC final)
    payload = frame[12:-4]

    data = np.frombuffer(payload, dtype=np.uint16).reshape(-1, 2)

    vin = data[:, 0]
    vout = data[:, 1]

    return vin, vout

def update():
    vin, vout = leer_trama()
    if vin is None:
        return

    curve_in.setData(vin)
    curve_out.setData(vout)

    fft = np.abs(np.fft.rfft(vout - np.mean(vout)))
    curve_fft.setData(fft)

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10)   # 10 ms = estable y no sobrecarga la CPU
def salir(*args):
    print("Cerrando...")
    ser.close()
    app.quit()
    sys.exit()

signal.signal(signal.SIGINT, salir)   # Ctrl+C
signal.signal(signal.SIGTERM, salir)  # kill

app.exec()

