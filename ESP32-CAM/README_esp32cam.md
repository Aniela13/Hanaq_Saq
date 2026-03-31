# Video ESP32-CAM — README

## Descripción

Módulo de transmisión de video en tiempo real desde el vehículo hacia la estación base (pantalla / laptop del equipo). Utiliza una **ESP32-CAM** con su Wi-Fi integrado y el protocolo **UDP** para minimizar la latencia y evitar que el video se congele ante pérdida de paquetes.

---

## ¿Por qué no usar XBee para el video?

El módulo XBee3 tiene una tasa de transferencia máxima de **250 kbps** en RF. Un frame de video comprimido JPEG a resolución baja (320×240) ocupa entre 8–20 KB, lo que tomaría entre 256–640 ms solo en transmisión RF. Esto hace imposible un video en tiempo real por XBee.

**Solución:** usar el Wi-Fi integrado de la ESP32-CAM, que opera a varios Mbps, suficiente para video fluido.

---

## ¿Por qué UDP en vez de HTTP?

| Protocolo | Comportamiento ante pérdida | Latencia | Uso típico |
|---|---|---|---|
| HTTP / TCP | Espera retransmisión del paquete perdido → video se **congela** | Alta | Descarga de archivos |
| UDP | Descarta el paquete perdido y sigue con el siguiente → video **continúa** | Baja | Streaming, videollamadas, juegos |

En un entorno con posible interferencia Wi-Fi (competencia con muchos equipos), perder algunos frames es aceptable. Lo que no es aceptable es que el video se congele varios segundos esperando una retransmisión TCP. UDP prioriza la fluidez sobre la integridad de cada frame.

---

## Hardware

| Componente | Descripción |
|---|---|
| ESP32-CAM (AI-Thinker) | Módulo con cámara OV2640 y Wi-Fi integrado |
| Cable FTDI / programador USB-UART | Para cargar el firmware (la ESP32-CAM no tiene USB propio) |
| Antena Wi-Fi (opcional) | Mejora el alcance si la competencia es en exterior |

---

## Arquitectura de la transmisión

```
[ESP32-CAM]
  Captura frame JPEG (OV2640)
  Fragmenta en paquetes UDP (<1400 bytes cada uno)
  Envía por Wi-Fi UDP al IP de la estación base
        │
        │  Wi-Fi 2.4 GHz
        ▼
[Laptop / Pantalla estación base]
  Recibe paquetes UDP
  Reensambla el frame
  Muestra en pantalla (Python + OpenCV / Processing / navegador)
```

---

## Parámetros de transmisión recomendados

| Parámetro | Valor recomendado | Razón |
|---|---|---|
| Resolución | 320×240 (QVGA) | Balance calidad/velocidad |
| Calidad JPEG | 10–20 (escala 0–63, menor = mejor calidad) | Reduce tamaño del frame |
| FPS objetivo | 15–20 fps | Suficiente para telemetría visual |
| Puerto UDP | 5005 | Configurable |
| MTU por paquete | 1400 bytes | Evita fragmentación IP |

---

## Comparación con otras opciones de video

| Opción | Ventaja | Desventaja |
|---|---|---|
| ESP32-CAM UDP (elegida) | Baja latencia, económica, simple | Alcance limitado al Wi-Fi |
| ESP32-CAM HTTP MJPEG | Fácil de ver en navegador | Alta latencia, se congela |
| Cámara analógica + transmisor 5.8GHz | Muy baja latencia, largo alcance | Costo mayor, no integrado con datos |
| XBee + imagen | — | Imposible en tiempo real (250 kbps) |

---

## Estado

- [ ] Código de captura y fragmentación UDP en ESP32-CAM
- [ ] Script receptor en estación base (Python + OpenCV)
- [ ] Prueba de alcance y latencia en exterior
- [ ] Integración con pantalla del equipo en competencia
