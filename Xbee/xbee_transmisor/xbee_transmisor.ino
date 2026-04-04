/**
 * XBee AT Mode — Transmisor ESP32
 * Datos: Accel(3), Gyro(3), GPS(lat,lon,alt,time), Bat, Hum, Temp, Presión
 * Paquete binario de 30 bytes para mínima latencia
 *
 * Conexión XBee:
 *   XBee TX  → ESP32 GPIO16 (RX2)
 *   XBee RX  → ESP32 GPIO17 (TX2)
 *   XBee VCC → 3.3V
 *   XBee GND → GND
 */

#include <Arduino.h>

// ── Configuración ────────────────────────────────────────────────────────────
#define XBEE_SERIAL   Serial2
#define XBEE_BAUD     115200
#define XBEE_RX_PIN   7
#define XBEE_TX_PIN   8

#define SEND_INTERVAL_MS  100   // 10 Hz → latencia máxima 100 ms
#define PACKET_HEADER     0xAA
#define PACKET_LEN        30    // bytes de payload (sin header, len, crc)

// ── Estructura de datos (desempaquetada) ─────────────────────────────────────
struct SensorData {
  // Aceleración m/s² × 100 → int16  (±327.67 m/s²)
  int16_t ax, ay, az;
  // Giroscopio °/s × 100 → int16  (±327.67 °/s)
  int16_t gx, gy, gz;
  // GPS
  int32_t lat;    // grados × 1e6  (ej. -12.046374 → -12046374)
  int32_t lon;    // grados × 1e6
  int32_t alt;    // altitud en cm
  uint16_t gtime; // segundos desde medianoche UTC (0–86399)
  // Ambiente
  uint8_t bat;    // porcentaje 0–100
  uint8_t hum;    // porcentaje 0–100
  int8_t  temp;   // temperatura: valor × 2  (±63.5 °C con 0.5 °C res.)
  uint8_t pres;   // presión: valor - 940  (940–1195 hPa → 0–255)
};

// ── CRC-8 Dallas/Maxim ───────────────────────────────────────────────────────
uint8_t crc8(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    uint8_t byte = *data++;
    for (uint8_t i = 0; i < 8; i++) {
      if ((crc ^ byte) & 0x80) crc = (crc << 1) ^ 0x31;
      else                      crc <<= 1;
      byte <<= 1;
    }
  }
  return crc;
}

// ── Serialización a buffer binario ──────────────────────────────────────────
// Retorna el tamaño total del paquete (header + len + payload + crc = 30)
uint8_t serializeSensorData(const SensorData& d, uint8_t* buf) {
  uint8_t i = 0;

  // Cabecera
  buf[i++] = PACKET_HEADER;
  buf[i++] = PACKET_LEN;

  // Aceleración (6 bytes, big-endian)
  buf[i++] = (d.ax >> 8) & 0xFF; buf[i++] = d.ax & 0xFF;
  buf[i++] = (d.ay >> 8) & 0xFF; buf[i++] = d.ay & 0xFF;
  buf[i++] = (d.az >> 8) & 0xFF; buf[i++] = d.az & 0xFF;

  // Giroscopio (6 bytes)
  buf[i++] = (d.gx >> 8) & 0xFF; buf[i++] = d.gx & 0xFF;
  buf[i++] = (d.gy >> 8) & 0xFF; buf[i++] = d.gy & 0xFF;
  buf[i++] = (d.gz >> 8) & 0xFF; buf[i++] = d.gz & 0xFF;

  // GPS lat (4 bytes)
  buf[i++] = (d.lat >> 24) & 0xFF; buf[i++] = (d.lat >> 16) & 0xFF;
  buf[i++] = (d.lat >>  8) & 0xFF; buf[i++] =  d.lat        & 0xFF;

  // GPS lon (4 bytes)
  buf[i++] = (d.lon >> 24) & 0xFF; buf[i++] = (d.lon >> 16) & 0xFF;
  buf[i++] = (d.lon >>  8) & 0xFF; buf[i++] =  d.lon        & 0xFF;

  // GPS alt (4 bytes)
  buf[i++] = (d.alt >> 24) & 0xFF; buf[i++] = (d.alt >> 16) & 0xFF;
  buf[i++] = (d.alt >>  8) & 0xFF; buf[i++] =  d.alt        & 0xFF;

  // GPS time (2 bytes)
  buf[i++] = (d.gtime >> 8) & 0xFF; buf[i++] = d.gtime & 0xFF;

  // Ambiente + batería (4 bytes)
  buf[i++] = d.bat;
  buf[i++] = d.hum;
  buf[i++] = (uint8_t)d.temp;
  buf[i++] = d.pres;

  // CRC-8 sobre el payload (buf[2] hasta buf[i-1])
  buf[i] = crc8(&buf[2], PACKET_LEN);
  i++;

  return i; // debe ser 30
}

// ── Generación de datos aleatorios lógicos ───────────────────────────────────
SensorData generateRandomData() {
  SensorData d;

  // Aceleración: ±2g en reposo (1g ≈ 9.81 m/s²), con ruido ±0.5
  d.ax = (int16_t)((random(-50, 50)) );           // ~0 m/s² lateral
  d.ay = (int16_t)((random(-50, 50)) );           // ~0 m/s² lateral
  d.az = (int16_t)(981 + random(-30, 30));        // ~9.81 m/s² (gravedad)

  // Giroscopio: reposo con pequeño drift
  d.gx = (int16_t)(random(-200, 200));            // ±2 °/s
  d.gy = (int16_t)(random(-200, 200));
  d.gz = (int16_t)(random(-200, 200));

  // GPS — Lima, Perú aproximado con pequeña variación
  d.lat = (int32_t)(-12046374 + random(-100, 100));
  d.lon = (int32_t)(-77042793 + random(-100, 100));
  d.alt = (int32_t)(15400 + random(-50, 50));     // ~154 m en cm
  d.gtime = (uint16_t)(millis() / 1000 % 86400); // segundos del día

  // Batería (simula descarga lenta: 100→20%)
  static uint8_t batLevel = 100;
  if (millis() % 5000 < SEND_INTERVAL_MS) batLevel = max(20, batLevel - 1);
  d.bat = batLevel;

  // Ambiente Lima: ~70% HR, ~20°C, ~1013 hPa
  d.hum  = (uint8_t)(70 + random(-5, 5));
  d.temp = (int8_t)((20 + random(-2, 3)) * 2);   // ×2 para 0.5°C resolución
  d.pres = (uint8_t)(73 + random(-3, 3));         // 1013 hPa → 1013-940=73

  return d;
}

// ── Estadísticas de latencia ─────────────────────────────────────────────────
static uint32_t packetsSent   = 0;
static uint32_t totalSendUs   = 0;

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("[TX] Iniciando transmisor XBee AT...");

  // XBEE_SERIAL.begin(XBEE_BAUD, SERIAL_8N1, XBEE_RX_PIN, XBEE_TX_PIN);
  XBEE_SERIAL.begin(XBEE_BAUD);

  randomSeed(millis()); // Semilla verdaderamente aleatoria (hardware RNG) para teensy

  Serial.println("[TX] Listo. Enviando cada " + String(SEND_INTERVAL_MS) + " ms");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
static uint32_t lastSend = 0;

void loop() {
  uint32_t now = millis();

  if (now - lastSend >= SEND_INTERVAL_MS) {
    lastSend = now;

    SensorData data = generateRandomData();

    uint8_t buf[30];
    uint8_t len = serializeSensorData(data, buf);
    Serial.print("Enviando paquete de longitud: ");
    Serial.println(len);
    // Medir tiempo de escritura al buffer UART
    uint32_t t0 = micros();
    XBEE_SERIAL.write(buf, len);
    // XBEE_SERIAL.println("HOLA");
    // XBEE_SERIAL.flush(); // Espera a que el UART termine de enviar
    uint32_t elapsed = micros() - t0;

    packetsSent++;
    totalSendUs += elapsed;

    // Debug cada 20 paquetes (~2 s)
    if (packetsSent % 20 == 0) {
      float avgMs = (totalSendUs / packetsSent) / 1000.0f;
      Serial.printf("[TX] Pkt #%lu | %d bytes | UART ~%.2f ms | Temp:%.1f°C Hum:%d%% Bat:%d%%\n",
        packetsSent, len, avgMs,
        (float)data.temp / 2.0f,
        data.hum,
        data.bat);
    }
  }
}
