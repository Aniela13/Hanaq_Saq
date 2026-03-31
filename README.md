# Proyecto Hanaq — Documentación General

## Descripción

Sistema embebido de telemetría y visión para competencia, compuesto por módulos de comunicación inalámbrica (XBee), sensores IMU/GPS/ambientales y transmisión de video en tiempo real.

---

## Arquitectura general del sistema

```
[Teensy 4.1 / ESP32]          [XBee TX] ----RF 2.4GHz----> [XBee RX]          [PC / Estación base]
  Sensores + GPS                Telemetría binaria             Recepción               Visualización
  IMU, Baro, Bat

[ESP32-CAM]  ----Wi-Fi UDP----> [Pantalla / Receptor]
  Cámara                         Video en tiempo real
```

---

## Elección de microcontrolador: ESP32 vs Teensy 4.1

La elección del microcontrolador principal afecta directamente el rendimiento del procesamiento de sensores y la estabilidad de la telemetría.

| Característica | ESP32 | Teensy 4.1 |
|---|---|---|
| Velocidad de CPU | 240 MHz | 600 MHz |
| Núcleos | 2 (multitarea RTOS) | 1 (monohilo ultrarrápido) |
| Ecosistema | ESP-IDF / Arduino | Teensyduino (compatible Arduino) |
| Wi-Fi / Bluetooth | Integrado | No integrado |
| Pines I/O | Limitado | 55 pines, 8 puertos seriales |
| Interrupciones Wi-Fi | Puede interrumpir el loop principal | No aplica |
| Uso recomendado | Proyectos que requieren conectividad inalámbrica | Cálculo intensivo, lectura de sensores a alta velocidad |

### ¿Por qué Teensy 4.1 para telemetría?

- El Wi-Fi de la ESP32 genera interrupciones de software que pueden retrasar la lectura de sensores y el envío UART al XBee, introduciendo jitter en la latencia.
- La Teensy 4.1 a 600 MHz ejecuta el loop de sensores sin interrupciones externas, garantizando una cadencia de muestreo estable.
- Con 8 puertos seriales nativos, la Teensy puede manejar simultáneamente el GPS (UART), el XBee (UART), el IMU (SPI/I2C) y un puerto de debug sin conflictos.

### ¿Por qué ESP32 para la cámara?

- La ESP32-CAM tiene Wi-Fi integrado y es el módulo más económico y compacto para transmisión de video.
- El video no requiere la precisión temporal de los sensores, por lo que las interrupciones Wi-Fi no son un problema crítico en este módulo.

---

## Módulos del sistema

| Módulo | Archivo README | Descripción |
|---|---|---|
| Telemetría XBee | [`README_xbee.md`](./README_xbee.md) | Comunicación RF, modos AT y API, protocolo binario |
| Video ESP32-CAM | [`README_esp32cam.md`](./README_esp32cam.md) | Transmisión de video por UDP sobre Wi-Fi |

---

## Estado del proyecto

- [x] Configuración XBee en modo AT (XCTU)
- [x] Protocolo binario de 30 bytes definido
- [x] Código transmisor ESP32/Teensy
- [x] Código receptor ESP32/Teensy
- [ ] Integración con sensores reales (IMU, GPS, barométrico)
- [ ] Transmisión de video ESP32-CAM UDP
- [ ] Integración y prueba en competencia
