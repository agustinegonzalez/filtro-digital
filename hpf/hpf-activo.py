#!/usr/bin/env python3
"""
filtro_hp_butterworth4_33k.py

Filtro activo pasa-alto Butterworth 4.º orden usando dos Sallen-Key (cada una biquad HP)
+ una etapa amplificadora final no inversora para obtener la ganancia total requerida.

Especificaciones (por defecto, modificables más abajo):
- fc = 33e3 Hz
- Amax = 3 dB (Butterworth tiene -3 dB en fc, por diseño)
- Ganancia total v/v = 12.25
- C1 = C2 = 1 nF por biquad (valor práctico; puedes cambiar)
- Rg para implementar K en Sallen-Key y Rg_gain para la etapa final (valores por defecto)
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from math import pi

# ---------------------------
# Parámetros modificables
# ---------------------------
fc = 33_000.0          # frecuencia de corte en Hz
overall_gain = 12.25   # ganancia total v/v (lineal)
C_chosen = 1e-9        # C1 = C2 por biquad (F). Valor por defecto: 1 nF
Rg_sk = 10_000.0       # Rg para calcular Rf de cada Sallen-Key (Ω)
Rg_gain = 10_000.0     # Rg para la etapa amplificadora final (Ω)

# ---------------------------
# Polos Butterworth normalizados (N=4)
# ---------------------------
N = 4
k = np.arange(1, N+1)
poles = np.exp(1j * (pi * (2*k + N - 1) / (2*N)))   # polos normalizados en el semiplano izquierdo
poles = poles[np.real(poles) < 0]                   # quedarnos con polos estables (parte real < 0)

# Para fc dado:
omega0 = 2.0 * pi * fc

sections = []
# Cada polo complejo (y su conjugado) forma una sección de 2. orden
for idx, p in enumerate(poles):
    # Q de la sección (prototipo normalizado)
    Q = -1.0 / (2.0 * np.real(p))
    # Con R1=R2=R y C1=C2=C_chosen: omega0 = 1/(R*C) -> R = 1/(omega0 * C)
    R = 1.0 / (omega0 * C_chosen)
    # Para SK con R1=R2, C1=C2: Q = 1/(3 - K) -> K = 3 - 1/Q
    K = 3.0 - 1.0 / Q
    # Convertir K a Rf (no inversor): K = 1 + Rf/Rg_sk  => Rf = (K-1)*Rg_sk
    Rf = (K - 1.0) * Rg_sk

    sections.append({
        "Sección": f"Sallen-Key {idx+1}",
        "Pares de polos": f"{p:.6g}, {np.conj(p):.6g}",
        "Q": float(Q),
        "omega0 (rad/s)": float(omega0),
        "fc (Hz)": float(fc),
        "C1=C2 (F)": float(C_chosen),
        "R1=R2 (Ω)": float(R),
        "K (SK)": float(K),
        "Rg_sk (Ω)": float(Rg_sk),
        "Rf_sk (Ω)": float(Rf)
    })

df = pd.DataFrame(sections)

# ---------------------------
# Etapa amplificadora final (no inversora) para conseguir overall_gain
# ---------------------------
# Ganancia de las biquads (a altas frecuencias) es ~1 en magnitud de la normalización;
# para asegurar que la ganancia global sea overall_gain, añadimos una etapa gain_stage con:
# K_gain = overall_gain  (no la repartimos entre SKs para no alterar Q)
K_gain = overall_gain
Rf_gain = (K_gain - 1.0) * Rg_gain

gain_stage = {
    "Etapa": "Amplificador final (no-inversor)",
    "K_gain (v/v)": float(K_gain),
    "Rg_gain (Ω)": float(Rg_gain),
    "Rf_gain (Ω)": float(Rf_gain)
}

# ---------------------------
# Función de transferencia del biquad HP (normalizado a omega0)
# ---------------------------
def biquad_hp_response(s, w0, Q):
    """Respuesta de un biquad high-pass normalizado (s domain).
       H(s) = (s/w0)^2 / [ (s/w0)^2 + (s/w0)/Q + 1 ]
    """
    x = s / w0
    return (x**2) / (x**2 + x / Q + 1.0)

# ---------------------------
# Calcular y graficar respuesta en frecuencia
# ---------------------------
# Barrido logaritmico de 1 Hz a 1 MHz (ajusta si quieres)
w = np.logspace(np.log10(2*pi*1.0), np.log10(2*pi*1e6), 2000)  # rad/s
s = 1j * w

H_total = np.ones_like(s, dtype=complex)
for sec in sections:
    H_total *= biquad_hp_response(s, omega0, sec["Q"])

# Aplicar la ganancia final
H_total *= K_gain

# Magnitud en dB
mag_db = 20.0 * np.log10(np.abs(H_total))

# ---------------------------
# Mostrar resultados
# ---------------------------
pd.set_option('display.float_format', lambda x: f"{x:.6g}")
print("\n=== Diseño: Butterworth 4º (HP) — especificaciones ===")
print(f"fc = {fc:.6g} Hz, Ganancia total (v/v) = {overall_gain:.6g}, Amax = 3 dB (Butterworth)\n")
print("Valores por sección Sallen-Key (R1=R2, C1=C2):\n")
print(df.to_string(index=False))
print("\nEtapa amplificadora final (para obtener la ganancia total):")
print(gain_stage)

# ---------------------------
# Graficar
# ---------------------------
plt.figure(figsize=(8,5))
plt.semilogx(w/(2*pi), mag_db)
plt.grid(True, which='both', ls=':', alpha=0.6)
plt.xlabel("Frecuencia (Hz)")
plt.ylabel("Magnitud (dB)")
plt.title(f"Filtro Pasa-Alto Butterworth 4º — fc={fc:.0f} Hz, Ganancia total={overall_gain}")
# marcar fc
plt.axvline(fc, color='gray', linestyle='--', linewidth=0.8)
# marcar Amax (nivel de paso) en dB: 20*log10(overall_gain)
passband_db = 20.0*np.log10(overall_gain)
plt.axhline(passband_db, color='green', linestyle='--', linewidth=0.8)
plt.text(fc*1.05, passband_db+1, f"Passband = {passband_db:.2f} dB", color='green')
plt.ylim(passband_db - 40, passband_db + 6)
plt.xlim(100, 2e6)
plt.tight_layout()
plt.show()

# ---------------------------
# Notas finales impresas
# ---------------------------
print("\nNOTAS:")
print("- El criterio Amax = 3 dB está implícito en la aproximación Butterworth (magnitud -3 dB en fc).")
print("- No repartí la ganancia entre las biquads para no alterar sus Q (la etapa final proporciona la ganancia global).")
print("- Si prefieres repartir la ganancia entre etapas (y ajustar R/C en consecuencia), se puede intentar,")
print("  pero cambiar K en una Sallen-Key altera su Q, por lo que habría que rediseñar las secciones.")
print("- Revisa el GBW y slew rate del op-amp elegido: fc = 33 kHz y la sumatoria de Q piden que el op-amp tenga GBW suficiente.")
print("- Si quieres que normalice resistencias a valores E12/E24 o use otros C, dímelo y actualizo el script.")

