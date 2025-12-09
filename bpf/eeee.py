import time
import serial, struct
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading

# === CONFIGURACION ===
PORT = "/dev/ttyACM0"
BAUD = 115200
MAGIC = b"\xA5\x5A\xA5\x5A"

# Frecuencia de Muestreo (100kHz)
FS_REAL = 100000.0  

VREF = 3.3
ADC_RES = 4095.0
PAUSADO = False

try:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    ser.reset_input_buffer()
    print(f"--- CONECTADO A {PORT} ---")
    print(f"Osciloscopio Digital (Zoom: 0.5 ms)")
    print("[ESPACIO]: Pausar/Reanudar")
except Exception as e:
    print(f"Error: {e}")
    exit()

# Variables Compartidas
data_vin = deque([2048]*4096, maxlen=4096)
data_vout = deque([2048]*4096, maxlen=4096)
rx_buffer = bytearray()

# --- HILO DE LECTURA ---
def leer_serial():
    global rx_buffer
    while True:
        try:
            if not ser.is_open: break
            
            if ser.in_waiting:
                rx_buffer.extend(ser.read(ser.in_waiting))
            
            # Proteccion anti-desbordamiento
            if len(rx_buffer) > 20000:
                del rx_buffer[:] 
                ser.reset_input_buffer()
                continue

            while len(rx_buffer) > 16:
                idx = rx_buffer.find(MAGIC)
                if idx == -1:
                    del rx_buffer[:-3]
                    break
                if idx > 0:
                    del rx_buffer[:idx]
                    continue
                
                if len(rx_buffer) < 12: break
                
                fs_dummy, n_samples, seq = struct.unpack("<IHH", rx_buffer[4:12])
                packet_size = 12 + (n_samples * 4) + 4
                
                if len(rx_buffer) < packet_size: break
                
                tail = rx_buffer[packet_size-4 : packet_size]
                if tail == MAGIC:
                    payload = rx_buffer[12 : 12 + n_samples*4]
                    raw_data = np.frombuffer(payload, dtype=np.uint16)
                    
                    new_vin = raw_data[0::2]
                    new_vout = raw_data[1::2]
                    
                    if not PAUSADO:
                        data_vin.extend(new_vin)
                        data_vout.extend(new_vout)
                
                del rx_buffer[:packet_size]
            
            time.sleep(0.005)
            
        except Exception as e:
            time.sleep(1)

t = threading.Thread(target=leer_serial)
t.daemon = True
t.start()

# --- GRAFICO TIEMPO ---
fig, ax1 = plt.subplots(figsize=(10, 6))

line_in, = ax1.plot([], [], 'b', label='Entrada', lw=1, alpha=0.6)
line_out, = ax1.plot([], [], 'r', label='Salida', lw=1.5)

ax1.set_title(f"Senal en Tiempo Real (Zoom: 0.5 ms)")
ax1.set_ylabel("Voltaje [V]")
ax1.set_ylim(-0.2, 3.5)
ax1.set_xlim(0, 0.5) 
ax1.grid(True)
ax1.legend(loc='upper right')
ax1.axhline(1.65, color='gray', linestyle=':')

def on_key(event):
    global PAUSADO
    if event.key == ' ':
        PAUSADO = not PAUSADO

fig.canvas.mpl_connect('key_press_event', on_key)

def update(frame):
    if PAUSADO: return line_in, line_out

    vin = np.array(data_vin) * (VREF / ADC_RES)
    vout = np.array(data_vout) * (VREF / ADC_RES)
    
    if len(vout) < 512: return line_in, line_out
    
    # --- DIBUJAR ---
    window_pts = int(0.5 * FS_REAL / 1000) 
    t_axis = np.arange(window_pts) / FS_REAL * 1000 
    
    try:
        line_in.set_data(t_axis, vin[-window_pts:])
        line_out.set_data(t_axis, vout[-window_pts:])
    except:
        pass

    return line_in, line_out

ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
plt.show()
