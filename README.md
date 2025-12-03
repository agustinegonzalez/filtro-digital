# Proceso de implementacion

1. Abrir STM32CubeIDE
2. Pegar el código C del ADC + UART + filtro
3. Compilar
4. Flashear la Bluepill con el ST-Link

La Bluepill queda funcionando como un "filtro digital en tiempo real".

# Conexion fisica

La signal del generador de funciones va a ir al PA0 (ADC IN) .

PA9 (TX) -> RX del convertidor USB
PA10(RX) <- TX del convertidor USB

La signal no debe exceder los 3.3V

# Resultados esperados
✔ La Bluepill recibe la señal analógica
✔ La filtra digitalmente
✔ La PC recibe ambas señales
✔ Python las grafica
