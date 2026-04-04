/**
 * XBee AT Mode — Receptor ESP32
 * Deserializa el paquete binario de 30 bytes y reconstruye los datos.
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
#define XBEE_RX_PIN   16
#define XBEE_TX_PIN   17

#define PACKET_HEADER 0xAA
#define PACKET_LEN    30
#define PACKET_TOTAL  33   // header(1) + len(1) + payload(28) + crc(1) = 31?
                           // Ajuste: header+len+payload+crc = 1+1+26+1 = 29... ver nota abajo
// Nota: la estructura exacta es:
//   [0]  0xAA header
//   [1]  28   (PACKET_LEN = bytes de payload)
//   [2..27]   26 bytes de payload  ← accel(6)+gyro(6)+lat(4)+lon(4)+alt(4)+time(2) = 26
//   [28]      batería
//   [29]      humedad
//   [30]      temperatura
//   [31] - pres  ← espera, revisamos la cuenta...
//
// Conteo real del TX:
//   accel  = 6, gyro = 6, lat = 4, lon = 4, alt = 4, time = 2 → 26 bytes
//   bat=1, hum=1, temp=1, pres=1 → 4 bytes
//   Total payload = 30 bytes  → PACKET_LEN = 30
//   Paquete total = 1+1+30+1 = 33 bytes
//
// Usamos las mismas constantes que el TX:
#undef PACKET_LEN
#undef PACKET_TOTAL
#define PACKET_LEN    30   // payload bytes (coincidir con TX)
#define PACKET_TOTAL  33   // 1(header)+1(len)+30(payload)+1(crc)

// ── Estructura de datos desempaquetada ───────────────────────────────────────
struct SensorData {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int32_t lat, lon, alt;
  uint16_t gtime;
  uint8_t bat, hum;
  int8_t  temp;
  uint8_t pres;
};

// ── CRC-8 (idéntico al TX) ───────────────────────────────────────────────────
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

// ── Deserialización ──────────────────────────────────────────────────────────
bool deserializeSensorData(const uint8_t* buf, SensorData& d) {
  // Verificar header
  if (buf[0] != PACKET_HEADER) return false;
  if (buf[1] != PACKET_LEN)    return false;

  // Verificar CRC (sobre buf[2..31])
  uint8_t receivedCrc = buf[PACKET_TOTAL - 1];
  uint8_t computedCrc = crc8(&buf[2], PACKET_LEN);
  if (receivedCrc != computedCrc) return false;

  uint8_t i = 2;

  // Aceleración
  d.ax = (int16_t)((buf[i] << 8) | buf[i+1]); i += 2;
  d.ay = (int16_t)((buf[i] << 8) | buf[i+1]); i += 2;
  d.az = (int16_t)((buf[i] << 8) | buf[i+1]); i += 2;

  // Giroscopio
  d.gx = (int16_t)((buf[i] << 8) | buf[i+1]); i += 2;
  d.gy = (int16_t)((buf[i] << 8) | buf[i+1]); i += 2;
  d.gz = (int16_t)((buf[i] << 8) | buf[i+1]); i += 2;

  // GPS lat
  d.lat  = ((int32_t)buf[i] << 24) | ((int32_t)buf[i+1] << 16)
          | ((int32_t)buf[i+2] << 8) | buf[i+3]; i += 4;

  // GPS lon
  d.lon  = ((int32_t)buf[i] << 24) | ((int32_t)buf[i+1] << 16)
          | ((int32_t)buf[i+2] << 8) | buf[i+3]; i += 4;

  // GPS alt
  d.alt  = ((int32_t)buf[i] << 24) | ((int32_t)buf[i+1] << 16)
          | ((int32_t)buf[i+2] << 8) | buf[i+3]; i += 4;

  // GPS time
  d.gtime = ((uint16_t)buf[i] << 8) | buf[i+1]; i += 2;

  // Ambiente
  d.bat  = buf[i++];
  d.hum  = buf[i++];
  d.temp = (int8_t)buf[i++];
  d.pres = buf[i++];

  return true;
}

// ── Impresión de datos ───────────────────────────────────────────────────────
void printSensorData(const SensorData& d, uint32_t pktNumber, uint32_t latencyUs) {
  Serial.println("─────────────────────────────────────");
  Serial.printf("Paquete #%lu  |  Latencia: %.2f ms\n", pktNumber, latencyUs / 1000.0f);
  Serial.printf("  Accel (m/s²)  X:%.2f  Y:%.2f  Z:%.2f\n",
    d.ax / 100.0f, d.ay / 100.0f, d.az / 100.0f);
  Serial.printf("  Gyro  (°/s)   X:%.2f  Y:%.2f  Z:%.2f\n",
    d.gx / 100.0f, d.gy / 100.0f, d.gz / 100.0f);
  Serial.printf("  GPS  Lat:%.6f  Lon:%.6f  Alt:%.2fm\n",
    d.lat / 1e6, d.lon / 1e6, d.alt / 100.0f);
  Serial.printf("  Hora UTC: %02u:%02u:%02u\n",
    d.gtime / 3600, (d.gtime % 3600) / 60, d.gtime % 60);
  Serial.printf("  Bat:%d%%  Hum:%d%%  Temp:%.1f°C  Pres:%d hPa\n",
    d.bat, d.hum, d.temp / 2.0f, (int)d.pres + 940);
}

// ── Buffer de recepción con máquina de estados ───────────────────────────────
// Patrón: busca el byte 0xAA y acumula el paquete completo.
// Evita búsquedas lentas (no usamos indexOf ni String).
static uint8_t rxBuf[PACKET_TOTAL];
static uint8_t rxIdx = 0;
static bool    inPacket = false;

static uint32_t packetsOk  = 0;
static uint32_t packetsBad = 0;
static uint32_t lastPacketUs = 0;

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("[RX] Iniciando receptor XBee AT...");

  XBEE_SERIAL.begin(XBEE_BAUD, SERIAL_8N1, XBEE_RX_PIN, XBEE_TX_PIN);

  // Aumentar buffer UART de recepción a 256 bytes para evitar overrun
  XBEE_SERIAL.setRxBufferSize(256);

  Serial.println("[RX] Esperando paquetes...");
  lastPacketUs = micros();
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  // Leer todos los bytes disponibles en este ciclo (no bloqueante)
  while (XBEE_SERIAL.available()) {
    uint8_t b = XBEE_SERIAL.read();
    Serial.printf("%02X ", b);
    if (!inPacket) {
      // Buscar byte de inicio
      if (b == PACKET_HEADER) {
        rxBuf[0] = b;
        rxIdx    = 1;
        inPacket = true;
      }
    } else {
      rxBuf[rxIdx++] = b;

      // Verificar longitud en byte[1]
      if (rxIdx == 2 && rxBuf[1] != PACKET_LEN) {
        // Longitud inválida → descartar y resetear
        inPacket = false;
        rxIdx    = 0;
        packetsBad++;
        continue;
      }

      // Paquete completo
      if (rxIdx >= PACKET_TOTAL) {
        uint32_t now = micros();
        uint32_t latency = now - lastPacketUs;
        lastPacketUs = now;

        SensorData data;
        if (deserializeSensorData(rxBuf, data)) {
          packetsOk++;
          printSensorData(data, packetsOk, latency);
          Serial.printf("  [OK: %lu  ERR: %lu  PER: %.2f%%]\n",
            packetsOk, packetsBad,
            100.0f * packetsBad / max(1UL, packetsOk + packetsBad));
        } else {
          packetsBad++;
          Serial.printf("[RX] ✗ Paquete inválido (CRC o header) | ERR=%lu\n", packetsBad);
        }

        // Resetear para el próximo paquete
        inPacket = false;
        rxIdx    = 0;
      }
    }
  }

  // Detección de timeout (>500 ms sin paquete)
  if (inPacket && (micros() - lastPacketUs) > 5000000UL) {
    // Serial.println("[RX] Timeout esperando paquete, reseteando buffer...");
    Serial.printf("[RX] Timeout. Bytes recibidos: %d de %d\n", rxIdx, PACKET_TOTAL);
    inPacket = false;
    rxIdx    = 0;
  }

  // if (XBEE_SERIAL.available()) {
  //   Serial.write(XBEE_SERIAL.read());
  // }

}
