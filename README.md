# STM32
En este repositorio está el control de sensores con la stm32.
Este es el "cerebro" del sistema, encargado de la lógica y la adquisición de datos. Firmware encargado de la lectura de sensores (humedad, lluvia, nivel, temp/hum) y ejecución del algoritmo de control para el actuador de la bomba.
Sección de Hardware:
  -Microcontrolador: STM32 (Librerías HAL).
  -Sensores: HL-69 (Humedad), DHT11 (Ambiente), HC-SR04 (Ultrasonido) y Sensor de Lluvia.
  -Actuador: Bomba de agua mediante relé con etapa de potencia BJT (2N2222).
