# Telemetría XBee — README

## Descripción

Módulo de comunicación inalámbrica para transmisión de datos de sensores en tiempo real entre el vehículo y la estación base. Utiliza módulos **Digi XBee3 Zigbee 3.0 TH** operando en **modo AT (transparente)**.

---

## Hardware

| Componente | Cantidad | Notas |
|---|---|---|
| XBee3 Zigbee 3.0 TH | 2 | Uno TX (Coordinator), uno RX (Router/End Device) |
| Adaptador USB-UART para XBee | 2 | Para configuración con XCTU |
| ESP32 o Teensy 4.1 | 2 | Microcontrolador en cada extremo |

### Conexión XBee ↔ Microcontrolador

```
XBee pin DOUT (TX) → MCU RX (GPIO16 en ESP32 / Serial1 en Teensy)
XBee pin DIN  (RX) → MCU TX (GPIO17 en ESP32 / Serial1 en Teensy)
XBee VCC           → 3.3V
XBee GND           → GND
```

---

## Modos de operación XBee

### Modo API

- La trama incluye dirección de destino, tipo de paquete y checksum de verificación.
- Los datos llegan completos o no llegan (verificación garantizada).
- Mayor latencia por procesamiento del checksum en cada trama.
- Requiere librería dedicada para armar y desarmar tramas (ej. `XBeeArduino`).
- Recomendado cuando la integridad de cada paquete es crítica y la latencia no es prioritaria.

### Modo AT — Transparente ✅ (modo elegido)

- El XBee actúa como un cable serial inalámbrico: todo lo que entra por UART sale por RF y viceversa.
- Latencia mínima posible — no hay procesamiento de tramas en el módulo.
- Implementación simple: se usa `Serial.write()` directamente, sin librerías externas.
- Desventaja: si hay interferencia RF, los bytes pueden llegar incompletos o corruptos.
- Mitigado con: **CRC-8 en el protocolo propio** + byte de header para sincronización.

### ¿Por qué AT para competencia?

En un entorno de competencia donde la latencia importa, el modo AT elimina el overhead de las tramas API (~10 bytes extra por paquete) y reduce el tiempo de procesamiento en el módulo. El control de errores se delega al protocolo binario propio implementado en el microcontrolador.

---

## Protocolo binario

En vez de enviar texto (JSON, CSV), se serializa cada campo a su representación binaria mínima. Esto reduce el paquete de ~140 bytes (JSON) a **30 bytes**, una reducción del 78%.

### Estructura del paquete (30 bytes)

```
Byte  0      : 0xAA          Header de sincronización
Byte  1      : 0x1E (30)     Longitud del payload
Bytes 2–7    : int16 × 3     Aceleración X, Y, Z  (×100, m/s²)
Bytes 8–13   : int16 × 3     Giroscopio X, Y, Z   (×100, °/s)
Bytes 14–17  : int32         Latitud GPS           (×1e6, grados)
Bytes 18–21  : int32         Longitud GPS          (×1e6, grados)
Bytes 22–25  : int32         Altitud GPS           (cm)
Bytes 26–27  : uint16        Tiempo GPS            (segundos desde medianoche)
Byte  28     : uint8         Batería               (0–100 %)
Byte  29     : uint8         Humedad               (0–100 %)
Byte  30     : int8          Temperatura           (valor × 2, °C)
Byte  31     : uint8         Presión               (valor − 940, hPa)
Byte  32     : uint8         CRC-8                 (Dallas/Maxim sobre bytes 2–31)
```

### Escalado de valores

| Campo | Tipo | Escala | Rango real |
|---|---|---|---|
| Aceleración | int16 | ×100 | ±327.67 m/s² |
| Giroscopio | int16 | ×100 | ±327.67 °/s |
| Latitud/Longitud | int32 | ×1 000 000 | ±180.000000° |
| Altitud | int32 | cm | ±21 474 km |
| Temperatura | int8 | ×2 | ±63.5 °C (res. 0.5°C) |
| Presión | uint8 | −940 | 940–1195 hPa |

---

## Configuración XCTU

### XBee Transmisor (Coordinator)

| Parámetro | Valor | Descripción |
|---|---|---|
| `CE` | 1 | Coordinator Enable |
| `ID` | PAN ID del equipo | Único por equipo en competencia |
| `DH` | `SH` del receptor | Dirección unicast |
| `DL` | `SL` del receptor | Dirección unicast |
| `BD` | 7 (115200) | Baud rate |
| `AP` | 0 | Modo AT |

### XBee Receptor (Router / End Device)

| Parámetro | Valor | Descripción |
|---|---|---|
| `CE` | 0 | End Device |
| `ID` | Mismo PAN ID | Debe coincidir con TX |
| `BD` | 7 (115200) | Mismo baud |
| `AP` | 0 | Modo AT |

> **Importante para competencia:** nunca usar `DL = FFFF` (broadcast). Usar siempre la dirección MAC exacta del receptor para evitar recibir datos de otros equipos.

---

## Latencia estimada

| Etapa | Tiempo aproximado |
|---|---|
| Serialización binaria en MCU | < 0.1 ms |
| Transmisión UART al XBee (30 bytes @ 115200) | ~2.6 ms |
| Propagación RF XBee3 | ~1–3 ms |
| Recepción UART + deserialización | ~0.5 ms |
| **Total estimado** | **~4–6 ms** |

Comparado con JSON en modo AT: ~12–15 ms. Con modo API: ~8–12 ms adicionales.

---

## Archivos de código

| Archivo | Descripción |
|---|---|
| `xbee_transmisor.ino` | Serialización y envío del paquete binario |
| `xbee_receptor.ino` | Recepción, verificación CRC y deserialización |
