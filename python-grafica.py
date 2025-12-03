import serial
import matplotlib.pyplot as plt

ser = serial.Serial('/dev/ttyUSB0', 115200)

xs = []
ys = []

while len(xs) < 5000:
    line = ser.readline().decode()
    x, y = map(float, line.split(','))
    xs.append(x)
    ys.append(y)

plt.plot(xs, label='Entrada')
plt.plot(ys, label='Salida filtrada')
plt.legend()
plt.show()

