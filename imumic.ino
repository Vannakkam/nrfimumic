/*
 * XIAO nRF52840 Sense — BLE IMU + Mic Streamer
 *
 * BLE Service UUID:  12345678-1234-5678-1234-56789abcdef0
 *   Time Sync Char : 12345678-1234-5678-1234-56789abcdef1  (Write, 8 bytes = uint64 epoch ms)
 *   Control Char   : 12345678-1234-5678-1234-56789abcdef2  (Write, 1 byte: 1=start, 0=stop)
 *   IMU Char       : 12345678-1234-5678-1234-56789abcdef3  (Notify)
 *   Mic Char       : 12345678-1234-5678-1234-56789abcdef4  (Notify)
 *
 * IMU : LSM6DS3 @ 416 Hz (accel + gyro, 16-bit signed)
 * Mic : MP34DT06TR-M PDM @ 20 kHz, 16-bit signed
 *
 * Timestamps are Unix epoch ms (set by the web client).
 */

#include <ArduinoBLE.h>
#include <Adafruit_LSM6DS.h>
#include <PDM.h>

// ---------- BLE UUIDs ----------
BLEService streamService("12345678-1234-5678-1234-56789abcdef0");

BLEByteCharacteristic controlChar("12345678-1234-5678-1234-56789abcdef2",
                                  BLERead | BLEWrite);
BLEUnsignedLongLongCharacteristic timeChar("12345678-1234-5678-1234-56789abcdef1",
                                           BLERead | BLEWrite);
BLECharacteristic imuChar("12345678-1234-5678-1234-56789abcdef3",
                          BLENotify, 120);   // up to 120 bytes per packet
BLECharacteristic micChar("12345678-1234-5678-1234-56789abcdef4",
                          BLENotify, 244);   // max ATT payload

// ---------- Timing / sync ----------
static uint64_t epochBaseMs   = 0;   // Unix epoch (ms) at the moment of sync
static uint32_t syncLocalMs   = 0;   // millis() at the moment of sync
static bool     timeSynced    = false;
static bool     streaming     = false;

static inline uint64_t nowEpochMs() {
  return epochBaseMs + (uint64_t)(millis() - syncLocalMs);
}

// ---------- IMU ----------
Adafruit_LSM6DS lsm6ds;
static uint32_t lastImuSampleMs = 0;
static const uint32_t IMU_PERIOD_US = 1000000UL / 416;  // ~2404 us

// ---------- Mic (PDM) ----------
static const int MIC_SAMPLE_RATE = 20000;
static const int MIC_BUFFER_SAMPLES = 200;           // 10 ms @ 20 kHz
static int16_t   pdmBuffer[MIC_BUFFER_SAMPLES];
static volatile int pdmSamplesReady = 0;
static int16_t   micRing[MIC_BUFFER_SAMPLES * 4];    // small ring for decoupling
static volatile int micRingHead = 0;
static volatile int micRingTail = 0;

// ---------- Packet buffers ----------
// IMU packet: [epochMs(8)][ax(2)][ay(2)][az(2)][gx(2)][gy(2)][gz(2)] = 20 bytes
static uint8_t imuPacket[20];

// Mic packet: [epochMs(8)][samples(N*2)]  -> up to 244 bytes -> 118 samples max
static uint8_t micPacket[244];

// ---------- Helpers ----------
static void writeU64(uint8_t *dst, uint64_t v) {
  for (int i = 0; i < 8; i++) dst[i] = (v >> (8 * i)) & 0xFF; // little-endian
}
static void writeI16(uint8_t *dst, int i, int16_t v) {
  dst[i] = v & 0xFF; dst[i + 1] = (v >> 8) & 0xFF;
}

// ---------- PDM callback ----------
void onPDMData(void) {
  int bytesAvailable = PDM.available();
  int samples = bytesAvailable / 2;
  if (samples > MIC_BUFFER_SAMPLES) samples = MIC_BUFFER_SAMPLES;
  PDM.read(pdmBuffer, samples * 2);

  // Push into ring (drop if full)
  for (int i = 0; i < samples; i++) {
    int next = (micRingHead + 1) % (MIC_BUFFER_SAMPLES * 4);
    if (next == micRingTail) { /* overflow, drop oldest by advancing tail */
      micRingTail = (micRingTail + 1) % (MIC_BUFFER_SAMPLES * 4);
    }
    micRing[micRingHead] = pdmBuffer[i];
    micRingHead = next;
  }
  pdmSamplesReady += samples;
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}
  Serial.println(F("XIAO nRF52840 Sense BLE streamer"));

  // IMU
  if (!lsm6ds.begin_I2C()) {
    Serial.println(F("LSM6DS not found!"));
    while (1) delay(10);
  }
  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  lsm6ds.setAccelDataRate(LSM6DS_RATE_416_HZ);
  lsm6ds.setGyroDataRate(LSM6DS_RATE_416_HZ);
  Serial.println(F("IMU OK @ 416 Hz"));

  // Mic
  if (!PDM.begin(1, MIC_SAMPLE_RATE)) {
    Serial.println(F("PDM init failed"));
    while (1) delay(10);
  }
  PDM.onReceive(onPDMData);
  Serial.println(F("Mic OK @ 20 kHz"));

  // BLE
  if (!BLE.begin()) {
    Serial.println(F("BLE init failed"));
    while (1) delay(10);
  }
  BLE.setLocalName("XIAO-Sense-IMU-MIC");
  BLE.setAdvertisedService(streamService);
  streamService.addCharacteristic(timeChar);
  streamService.addCharacteristic(controlChar);
  streamService.addCharacteristic(imuChar);
  streamService.addCharacteristic(micChar);
  BLE.addService(streamService);
  BLE.advertise();
  Serial.println(F("BLE advertising..."));

  controlChar.setEventHandler(BLEWritten, onControlWrite);
  timeChar.setEventHandler(BLEWritten, onTimeWrite);
}

void onTimeWrite(BLEDevice central, BLECharacteristic ch) {
  uint8_t buf[8] = {0};
  int len = ch.valueLength();
  if (len >= 8) {
    ch.readValue(buf, 8);
    uint64_t t = 0;
    for (int i = 0; i < 8; i++) t |= ((uint64_t)buf[i]) << (8 * i);
    epochBaseMs = t;
    syncLocalMs = millis();
    timeSynced = true;
    Serial.print(F("Time synced: ")); Serial.println((unsigned long)t);
  }
}

void onControlWrite(BLEDevice central, BLECharacteristic ch) {
  uint8_t v = 0;
  ch.readValue(&v, 1);
  if (v == 1) {
    streaming = true;
    lastImuSampleMs = micros() / 1000;
    Serial.println(F("Streaming START"));
  } else {
    streaming = false;
    Serial.println(F("Streaming STOP"));
  }
}

// ---------- Main loop ----------
void loop() {
  BLE.poll();

  if (!streaming || !timeSynced) return;

  // ---- IMU @ 416 Hz (software-timed from LSM6DS FIFO-less reads) ----
  uint32_t nowUs = micros();
  uint32_t nowMs = nowUs / 1000;
  if ((nowMs - lastImuSampleMs) >= (IMU_PERIOD_US / 1000)) {
    lastImuSampleMs = nowMs;

    float ax, ay, az, gx, gy, gz;
    lsm6ds.readAcceleration(ax, ay, az);
    lsm6ds.readGyroscope(gx, gy, gz);

    // Convert to raw-ish 16-bit (scale preserved as float in packet would need
    // more bytes; here we send engineering units as fixed-point *1000).
    int16_t rax = (int16_t)(ax * 1000.0f);
    int16_t ray = (int16_t)(ay * 1000.0f);
    int16_t raz = (int16_t)(az * 1000.0f);
    int16_t rgx = (int16_t)(gx * 100.0f);  // 0.01 dps resolution
    int16_t rgy = (int16_t)(gy * 100.0f);
    int16_t rgz = (int16_t)(gz * 100.0f);

    writeU64(imuPacket, nowEpochMs());
    writeI16(imuPacket, 8,  rax);
    writeI16(imuPacket, 10, ray);
    writeI16(imuPacket, 12, raz);
    writeI16(imuPacket, 14, rgx);
    writeI16(imuPacket, 16, rgy);
    writeI16(imuPacket, 18, rgz);

    if (imuChar.subscribed()) imuChar.writeValue(imuPacket, sizeof(imuPacket));
  }

  // ---- Mic: send 10 ms chunks (200 samples) when ready ----
  static uint32_t lastMicMs = 0;
  uint32_t curMs = millis();
  if ((curMs - lastMicMs) >= 10 && pdmSamplesReady >= MIC_BUFFER_SAMPLES) {
    lastMicMs = curMs;

    int take = MIC_BUFFER_SAMPLES;
    writeU64(micPacket, nowEpochMs());
    int off = 8;
    for (int i = 0; i < take && off + 1 < (int)sizeof(micPacket); i++) {
      int16_t s = micRing[micRingTail];
      micRingTail = (micRingTail + 1) % (MIC_BUFFER_SAMPLES * 4);
      micPacket[off++] = s & 0xFF;
      micPacket[off++] = (s >> 8) & 0xFF;
    }
    pdmSamplesReady -= take;
    if (pdmSamplesReady < 0) pdmSamplesReady = 0;

    if (micChar.subscribed()) micChar.writeValue(micPacket, off);
  }
}
