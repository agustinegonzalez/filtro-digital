#Filtro filterType IIR (infinite impulse response)
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
from scipy.signal import iirfilter

def design_filter(fs,fc,gain,order,filterType,btype,Amax):
    """
    fs:f sampling
    fc:f corte
    """
    fc_normalized = fc/(fs/2)
    
    
    if fc_normalized >= 1:
        raise ValueError("La frecuencia de corte debe ser menor que fs/2")
    
    # Diseño sedgún el filterType
    if filterType == 'butterworth':
        b, a = signal.butter(order, fc_normalized, btype)
    elif filterType == 'chebyshev':
        b, a = signal.cheby1(order, Amax, fc_normalized, btype)
    elif filterType == 'elliptic':
        b, a = signal.ellip(order, 1.0, 40, fc_normalized, btype)
    else:
        raise ValueError("Tipo de filtro no reconocido")
    
    # Aplicar gain
    b = b * gain
    
    return b, a

def visualizer(b, a, fs, fc):
    """
    Visualiza la respuesta en frecuencia del filtro
    """
    # Respuesta en frecuencia
    w, h = signal.freqz(b, a, worN=8000)
    H_passband = np.abs(h[-1])
    gain_passband_db = 20 * np.log10(H_passband)
    print("Ganancia en banda pasante [dB]:", gain_passband_db)

    # Convertir a frecuencia real (Hz)
    frequencys = w * fs / (2 * np.pi)

    # Magnitud en dB
    magnitud_db = 20 * np.log10(np.abs(h))

    # Fase en grados
    fase = np.unwrap(np.angle(h)) * 180/np.pi

    # Crear figura
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    # Gráfico de magnitud
    ax1.semilogx(frequencys, magnitud_db)
    ax1.set_title('Respuesta en Frecuencia del Filtro ')
    ax1.set_ylabel('Magnitud [dB]')
    ax1.set_xlabel('Frecuencia [Hz]')
    ax1.grid(True, which='both', alpha=0.3)
    ax1.axvline(fc, color='red', linestyle='--', label=f'Fc = {fc} Hz')
    ax1.axhline(-3, color='green', linestyle='--', label='-3 dB')
    ax1.legend()
    ax1.set_xlim([100, fs/2])

    # Gráfico de fase
    ax2.semilogx(frequencys, fase)
    ax2.set_ylabel('Fase [grados]')
    ax2.set_xlabel('Frecuencia [Hz]')
    ax2.grid(True, which='both', alpha=0.3)
    ax2.set_xlim([100, fs/2])

    plt.tight_layout()
    plt.show()

    return frequencys, magnitud_db, fase
class Filtro:
    def __init__(self, b, a):
        self.b = np.array(b)
        self.a = np.array(a)
        self.N = max(len(a), len(b))
        self.w = np.zeros(self.N - 1)

    def sample_processing(self, x):
        # Direct Form II Transposed
        y = self.b[0] * x + self.w[0]

        for i in range(1, self.N - 1):
            self.w[i - 1] = self.b[i] * x + self.w[i] - self.a[i] * y

        self.w[self.N - 2] = self.b[self.N - 1] * x - self.a[self.N - 1] * y

        return y

    def signal_processing(self, signal):
        out = np.zeros_like(signal)
        for i, x in enumerate(signal):
            out[i] = self.sample_processing(x)
        return out

def ejemplo_aplicacion():
    # Generar señal de prueba
    fs = 600000
    dur = 0.001  # 2 ms
    t = np.linspace(0, dur, int(fs * dur), endpoint=False) 
    f1 = 2400      # 5 kHz (DEBE atenuarse)
    f2 = 50000     # 50 kHz (DEBE pasar)

    señal = np.sin(2*np.pi*f1*t) + 0.5*np.sin(2*np.pi*f2*t)
    # Diseñar filtro (corte en  Hz)
    b, a = design_filter(fs, fc=33000, gain=10**(12/20), order=4,filterType='chebyshev',btype='high',Amax=3.0)
    
    # Crear filtro
    filtro = Filtro(b, a)
    
    # Procesar señal
    señal_filtrada = filtro.signal_processing(señal)
    
    # Visualizar resultados
    plt.figure(figsize=(12, 6))
    
    plt.subplot(2, 1, 1)
    plt.plot(t, señal)
    plt.title('Señal Original')
    plt.xlabel('Tiempo [s]')
    plt.ylabel('Amplitud')
    
    plt.subplot(2, 1, 2)
    plt.plot(t, señal_filtrada)
    plt.title('Señal Filtrada')
    plt.xlabel('Tiempo [s]')
    plt.ylabel('Amplitud')
    ax = plt.gca()

    # Divisiones principales cada 10 us
    ax.xaxis.set_major_locator(plt.MultipleLocator(10e-6))

    # Divisiones secundarias cada 2 us
    ax.xaxis.set_minor_locator(plt.MultipleLocator(2e-6))

    ax.grid(True, which='both', linestyle='--', alpha=0.5)

    # Hacer zoom para contar ciclos
    plt.xlim(0, 100e-6)   # 100 microsegundos
     
    plt.tight_layout()
    plt.show()

#b, a = iirfilter(4, Wn=33000/(500000/2), btype='highpass')
#print("b =", b)
#print("a =", a)
# Ejecutar ejemplo
ejemplo_aplicacion()

# Ejemplo de uso
if __name__ == "__main__":
    # Especificaciones
    fs = 200e3   
    fc = 33e3 
    gain = 12.5  # Ganancia de 2 (6 dB)
    order = 4
    btype = 'high'
    Amax = 3.0
    # Diseñar filtro
    b, a = design_filter(fs, fc, gain, order, 'butterworth',btype,Amax)

    print("Coeficientes del filtro:")
    print(f"Numerador b: {b}")
    print(f"Denominador a: {a}")

    # Visualizar
    frequencys, magnitud_db, fase = visualizer(b, a, fs, fc)
