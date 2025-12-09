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
    1_000_000,
    timeout=0
)

app = QtWidgets.QApplication([])

# ------------------------
# Graficas
# ------------------------
win = pg.GraphicsLayoutWidget(title="ADC + Filtro IIR en tiempo real")
p1 = win.addPlot(title="Entrada (VIN)")
curve_in = p1.plot(pen='y')

win.nextRow()
p2 = win.addPlot(title="Salida filtrada (VOUT)")
curve_out = p2.plot(pen='c')

# Configurar ejes Y en volts 0..3.3
p1.setYRange(0, 3.3)
p2.setYRange(0, 3.3)
ticks_v = [(v, f"{v:.1f}") for v in np.arange(0, 3.31, 0.1)]
p1.getAxis('left').setTicks([ticks_v])
p2.getAxis('left').setTicks([ticks_v])

win.show()

buffer = bytearray()

# Fs por defecto hasta recibir tramas
fs_actual = 200000

# Constante para convertir counts ADC → volts
ADC_TO_V = 3.3 / 4095.0


def leer_trama():
    global buffer, fs_actual

    data = ser.read(ser.in_waiting or 1)
    if data:
        buffer.extend(data)

    # Evitar crecimiento excesivo
    if len(buffer) > 200000:
        buffer = buffer[-50000:]

    idx = buffer.find(MAGIC)
    if idx == -1:
        return None, None, None

    header_size = 4 + 4 + 2 + 2
    if len(buffer) < idx + header_size:
        return None, None, None

    p = idx + 4
    fs = struct.unpack("<I", buffer[p:p+4])[0]; p += 4
    n  = struct.unpack("<H", buffer[p:p+2])[0]; p += 2
    seq = struct.unpack("<H", buffer[p:p+2])[0]; p += 2

    fs_actual = fs

    payload_size = n * 4
    total_size = header_size + payload_size + 4

    if len(buffer) < idx + total_size:
        return None, None, None

    frame = buffer[idx : idx + total_size]
    del buffer[: idx + total_size]

    payload = frame[12:-4]

    data = np.frombuffer(payload, dtype=np.uint16).reshape(-1, 2)

    vin = data[:, 0] * ADC_TO_V        # → convertir a voltios
    vout = data[:, 1] * ADC_TO_V

    return vin, vout, n


def update():
    global fs_actual
    vin, vout, n = leer_trama()
    if vin is None:
        return

    t = np.arange(n) * (1e6 / fs_actual)

    MAXPLOT = 100

    curve_in.setData(t[-MAXPLOT:], vin[-MAXPLOT:])
    curve_out.setData(t[-MAXPLOT:], vout[-MAXPLOT:])

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(1)


def salir(*args):
    print("Cerrando...")
    ser.close()
    app.quit()
    sys.exit()


signal.signal(signal.SIGINT, salir)
signal.signal(signal.SIGTERM, salir)

app.exec()

