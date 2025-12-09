import time
import serial, struct
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading

PORT = "/dev/ttyACM0"
BAUD = 1_000_000 
MAGIC = b"\xA5\x5A\xA5\x5A"

FS_REAL = 300000.0
VREF = 3.3
ADC_RES = 4095.0

PAUSADO = False          # pausa lectura
ANIM_STOP = False        # stop real de gráfica

COLOR_IN = "#4FA3F7"
COLOR_OUT = "#FF3366"
GRID_COLOR = "#AAAAAA"
BG_COLOR = "#1E1E1E"

try:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    ser.reset_input_buffer()
    print(f"✓ Conectado a {PORT}")
except Exception as e:
    print(f"Error: {e}")
    exit()

data_vin = deque([2048]*4096, maxlen=4096)
data_vout = deque([2048]*4096, maxlen=4096)
rx_buffer = bytearray()
FS_FROM_PACKET = FS_REAL

# -----------------------------------------------------------
#  HILO DE LECTURA SERIAL
# -----------------------------------------------------------
def leer_serial():
    global rx_buffer, FS_FROM_PACKET
    
    while True:
        try:
            if ser.in_waiting:
                rx_buffer.extend(ser.read(ser.in_waiting))

            if len(rx_buffer) > 20000:
                rx_buffer.clear()
                ser.reset_input_buffer()
                continue

            while len(rx_buffer) >= 12:
                idx = rx_buffer.find(MAGIC)
                if idx == -1:
                    del rx_buffer[:-3]
                    break
                if idx > 0:
                    del rx_buffer[:idx]
                    continue
                if len(rx_buffer) < 12:
                    break

                fs_val, n_samples, seq = struct.unpack("<IHH", rx_buffer[4:12])
                packet_size = 12 + n_samples*4 + 4

                if len(rx_buffer) < packet_size:
                    break

                if rx_buffer[packet_size-4:packet_size] == MAGIC:
                    payload = rx_buffer[12:12+n_samples*4]
                    raw = np.frombuffer(payload, dtype=np.uint16)

                    FS_FROM_PACKET = fs_val

                    if not PAUSADO:
                        data_vin.extend(raw[0::2])
                        data_vout.extend(raw[1::2])

                del rx_buffer[:packet_size]

            time.sleep(0.004)

        except Exception:
            time.sleep(0.5)


t = threading.Thread(target=leer_serial, daemon=True)
t.start()

# -----------------------------------------------------------
#  GRAFICO
# -----------------------------------------------------------
plt.style.use("dark_background")
fig, ax = plt.subplots(figsize=(11, 6))
fig.patch.set_facecolor(BG_COLOR)
ax.set_facecolor(BG_COLOR)

line_in, = ax.plot([], [], COLOR_IN, lw=1.2, label="Entrada VIN")
line_out, = ax.plot([], [], COLOR_OUT, lw=1.5, label="Salida VOUT")

ax.grid(True, color=GRID_COLOR, alpha=0.25)
ax.set_ylim(-0.2, 3.5)
ax.set_xlim(0, 0.5)
ax.set_xlabel("Tiempo (ms)")
ax.set_ylabel("Voltaje [V]")
ax.set_title("Señal en Tiempo Real (pasabanda digital)")
ax.legend(loc="upper right")

# -----------------------------------------------------------
#  TECLAS
# -----------------------------------------------------------
def on_key(event):
    global PAUSADO, ANIM_STOP, ani

    if event.key == " ":
        PAUSADO = not PAUSADO
        print("→ PAUSA" if PAUSADO else "→ EN VIVO")

    elif event.key.lower() == "s":
        ANIM_STOP = True
        ani.event_source.stop()
        print("→ ANIMACIÓN DETENIDA (FREEZE)")

    elif event.key.lower() == "r":
        ANIM_STOP = False
        ani.event_source.start()
        print("→ ANIMACIÓN RESUMIDA")

fig.canvas.mpl_connect("key_press_event", on_key)

def update(frame):

    if ANIM_STOP:
        return line_in, line_out

    if PAUSADO:
        return line_in, line_out

    fs = FS_FROM_PACKET

    vin = np.array(data_vin)*(VREF/ADC_RES)
    vout = np.array(data_vout)*(VREF/ADC_RES)

    if len(vout) < 100:
        return line_in, line_out

    window_pts = int(0.2 * fs / 1000)
    t_axis = (np.arange(window_pts)/fs)*1000

    line_in.set_data(t_axis, vin[-window_pts:])
    line_out.set_data(t_axis, vout[-window_pts:])

    ax.set_xlim(t_axis[0], t_axis[-1])
    ax.set_title(f"Señal en Tiempo Real — Fs={fs/1000:.1f} kHz")

    return line_in, line_out

ani = animation.FuncAnimation(fig, update, interval=40, blit=False)
plt.tight_layout()
plt.show()

