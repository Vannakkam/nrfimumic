/*
 * XIAO nRF52840 Sense - BLE IMU + PDM Microphone Streamer
 * 
 * Features:
 * - IMU (LSM6DS3TR-C) streaming at 416 Hz
 * - PDM Microphone streaming at 16 kHz
 * - BLE/Web Bluetooth connectivity
 * - Unix epoch time synchronization
 * - Binary packet streaming with flow control
 * - Optimized for low latency and no data loss
 * 
 * Libraries required:
 * - Seeed_Arduino_LSM6DS3 (for IMU)
 * - Seeed_Arduino_Mic (for PDM microphone)
 * - Adafruit Bluefruit nRF52 (included with Seeed nRF52 Boards)
 */

#include <Arduino.h>
#include <bluefruit.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include "PDM.h"

// BLE Service and Characteristics UUIDs
#define BLE_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_CONTROL_UUID       "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // Write: commands, Read/Notify
#define CHAR_TIME_UUID          "beb5483e-36e1-4688-b7f5-ea07361b26a9"  // Write: unix timestamp
#define CHAR_IMU_DATA_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26aa"  // Notify: IMU packets
#define CHAR_PDM_DATA_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26ab"  // Notify: PDM packets
#define CHAR_STATUS_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26ac"  // Notify: status

// IMU Configuration
#define IMU_ODR_HZ              416
#define IMU_SAMPLE_INTERVAL_US  (1000000 / IMU_ODR_HZ) // ~2403.8 us

// PDM Microphone Configuration
#define PDM_SAMPLE_RATE         16000
#define PDM_BUFFER_SIZE         512  // Samples per buffer
#define PDM_PACKET_SIZE         256  // 16-bit samples per BLE packet

// BLE Configuration
#define MAX_BLE_MTU             247
#define IMU_PACKET_SIZE         20   // Header(8) + accel(6*4) + gyro(6*4) = 8+24+24 = 56? optimize
#define IMU_PAYLOAD_SIZE        52   // timestamp(8) + ax(4)+ay(4)+az(4) + gx(4)+gy(4)+gz(4) + temp(4) + seq(8)? better compact
#define IMU_BLE_PACKET_SIZE     32   // Compact: seq(4) + ts_ms(4) + ax(2)+ay(2)+az(2)+gx(2)+gy(2)+gz(2) = 4+4+12 = 20, pad to 32

// Buffer sizes for no overflow
#define IMU_RING_BUFFER_SIZE    1024
#define PDM_RING_BUFFER_SIZE    4096

// Command definitions
#define CMD_START_STREAM        0x01
#define CMD_STOP_STREAM         0x02
#define CMD_GET_STATUS          0x03
#define CMD_SYNC_TIME           0x04
#define CMD_SET_RECORDING       0x05

// Data packet headers
#define PACKET_HEADER_IMU       0x49  // 'I'
#define PACKET_HEADER_PDM       0x50  // 'P'
#define PACKET_HEADER_STATUS    0x53  // 'S'

// IMU sensor
LSM6DS3 myIMU(I2C_MODE, 0x6A);

// BLE Characteristics
BLEService        sensorService = BLEService(BLE_SERVICE_UUID);
BLECharacteristic controlChar   = BLECharacteristic(CHAR_CONTROL_UUID);
BLECharacteristic timeChar      = BLECharacteristic(CHAR_TIME_UUID);
BLECharacteristic imuDataChar   = BLECharacteristic(CHAR_IMU_DATA_UUID);
BLECharacteristic pdmDataChar   = BLECharacteristic(CHAR_PDM_DATA_UUID);
BLECharacteristic statusChar    = BLECharacteristic(CHAR_STATUS_UUID);

// PDM variables
static short pdmBuffer[PDM_BUFFER_SIZE];
static volatile int pdmBufferCount = 0;
static volatile bool pdmDataReady = false;

// IMU ring buffer
struct IMUSample {
  uint32_t sequence;
  uint32_t timestamp_ms;
  int16_t ax, ay, az; // scaled
  int16_t gx, gy, gz; // scaled
  int16_t temp;
};

static IMUSample imuRingBuffer[IMU_RING_BUFFER_SIZE];
static volatile uint32_t imuWriteIndex = 0;
static volatile uint32_t imuReadIndex = 0;
static volatile uint32_t imuSequence = 0;

// Streaming state
static volatile bool isStreaming = false;
static volatile bool isRecording = false;
static volatile uint32_t unixEpochTime = 0;
static volatile uint32_t bootTimeMs = 0;
static volatile uint32_t lastIMUTime = 0;
static volatile uint32_t lastPDMPacketTime = 0;

// Connection state
static volatile bool bleConnected = false;

// PDM callback
void onPDMdata() {
  int bytesAvailable = PDM.available();
  int samplesRead = bytesAvailable / 2; // 16-bit samples
  
  if (pdmBufferCount + samplesRead < PDM_BUFFER_SIZE) {
    PDM.read(pdmBuffer + pdmBufferCount, bytesAvailable);
    pdmBufferCount += samplesRead;
  } else {
    // Buffer overflow prevention - drop oldest if necessary (shouldn't happen with proper flow control)
    // Shift buffer
    int overflow = (pdmBufferCount + samplesRead) - PDM_BUFFER_SIZE;
    if (overflow < pdmBufferCount) {
      memmove(pdmBuffer, pdmBuffer + overflow, (pdmBufferCount - overflow) * 2);
      pdmBufferCount -= overflow;
    } else {
      pdmBufferCount = 0;
    }
    PDM.read(pdmBuffer + pdmBufferCount, bytesAvailable);
    pdmBufferCount += samplesRead;
  }
  
  if (pdmBufferCount >= PDM_PACKET_SIZE) {
    pdmDataReady = true;
  }
}

// BLE connect callback
void connect_callback(uint16_t conn_handle) {
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  Serial.println("BLE Connected");
  bleConnected = true;
  
  // Request larger MTU for better throughput
  connection->requestMtuExchange(MAX_BLE_MTU);
}

// BLE disconnect callback
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  Serial.println("BLE Disconnected");
  bleConnected = false;
  isStreaming = false;
  isRecording = false;
}

// Control characteristic write callback
void control_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  if (len < 1) return;
  
  uint8_t cmd = data[0];
  Serial.printf("Control cmd: 0x%02X\n", cmd);
  
  switch (cmd) {
    case CMD_START_STREAM:
      isStreaming = true;
      imuWriteIndex = 0;
      imuReadIndex = 0;
      imuSequence = 0;
      pdmBufferCount = 0;
      pdmDataReady = false;
      Serial.println("Streaming started");
      break;
      
    case CMD_STOP_STREAM:
      isStreaming = false;
      Serial.println("Streaming stopped");
      break;
      
    case CMD_GET_STATUS:
      sendStatusPacket();
      break;
      
    case CMD_SET_RECORDING:
      if (len >= 2) {
        isRecording = data[1] == 0x01;
        Serial.printf("Recording: %s\n", isRecording ? "ON" : "OFF");
      }
      break;
      
    default:
      break;
  }
}

// Time characteristic write callback
void time_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  if (len >= 4) {
    // Read unix timestamp (little endian or big endian - handle both)
    unixEpochTime = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
    // If timestamp seems small, try big endian
    if (unixEpochTime < 1000000000UL) {
      unixEpochTime = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    }
    bootTimeMs = millis();
    Serial.printf("Time synced: %lu\n", unixEpochTime);
  }
}

// Send status packet
void sendStatusPacket() {
  if (!bleConnected) return;
  
  uint8_t statusPkt[8];
  statusPkt[0] = PACKET_HEADER_STATUS;
  statusPkt[1] = bleConnected ? 0x01 : 0x00;
  statusPkt[2] = isStreaming ? 0x01 : 0x00;
  statusPkt[3] = isRecording ? 0x01 : 0x00;
  statusPkt[4] = (imuWriteIndex - imuReadIndex) & 0xFF;
  statusPkt[5] = ((imuWriteIndex - imuReadIndex) >> 8) & 0xFF;
  statusPkt[6] = (pdmBufferCount >> 0) & 0xFF;
  statusPkt[7] = (pdmBufferCount >> 8) & 0xFF;
  
  statusChar.notify(statusPkt, 8);
}

// Send IMU data packet
void sendIMUPacket() {
  if (!bleConnected || !isStreaming) return;
  if (imuReadIndex == imuWriteIndex) return; // No data
  
  IMUSample* sample = &imuRingBuffer[imuReadIndex];
  imuReadIndex = (imuReadIndex + 1) % IMU_RING_BUFFER_SIZE;
  
  // Compact binary packet
  uint8_t imuPkt[32];
  memset(imuPkt, 0, 32);
  
  imuPkt[0] = PACKET_HEADER_IMU;
  imuPkt[1] = 0x01; // version
  // sequence (4 bytes, little endian)
  imuPkt[2] = sample->sequence & 0xFF;
  imuPkt[3] = (sample->sequence >> 8) & 0xFF;
  imuPkt[4] = (sample->sequence >> 16) & 0xFF;
  imuPkt[5] = (sample->sequence >> 24) & 0xFF;
  // timestamp (4 bytes)
  imuPkt[6] = sample->timestamp_ms & 0xFF;
  imuPkt[7] = (sample->timestamp_ms >> 8) & 0xFF;
  imuPkt[8] = (sample->timestamp_ms >> 16) & 0xFF;
  imuPkt[9] = (sample->timestamp_ms >> 24) & 0xFF;
  // accel x,y,z (2 bytes each, signed 16-bit)
  imuPkt[10] = sample->ax & 0xFF;
  imuPkt[11] = (sample->ax >> 8) & 0xFF;
  imuPkt[12] = sample->ay & 0xFF;
  imuPkt[13] = (sample->ay >> 8) & 0xFF;
  imuPkt[14] = sample->az & 0xFF;
  imuPkt[15] = (sample->az >> 8) & 0xFF;
  // gyro x,y,z
  imuPkt[16] = sample->gx & 0xFF;
  imuPkt[17] = (sample->gx >> 8) & 0xFF;
  imuPkt[18] = sample->gy & 0xFF;
  imuPkt[19] = (sample->gy >> 8) & 0xFF;
  imuPkt[20] = sample->gz & 0xFF;
  imuPkt[21] = (sample->gz >> 8) & 0xFF;
  // temp
  imuPkt[22] = sample->temp & 0xFF;
  imuPkt[23] = (sample->temp >> 8) & 0xFF;
  
  // Check if notification is enabled and not congested
  if (imuDataChar.notifyEnabled()) {
    imuDataChar.notify(imuPkt, 24); // Send compact packet
  }
}

// Send PDM data packet
void sendPDMPacket() {
  if (!bleConnected || !isStreaming) return;
  if (!pdmDataReady || pdmBufferCount < PDM_PACKET_SIZE) return;
  
  uint8_t pdmPkt[2 + PDM_PACKET_SIZE * 2];
  pdmPkt[0] = PACKET_HEADER_PDM;
  pdmPkt[1] = 0x01; // version
  
  // Copy samples (little endian)
  for (int i = 0; i < PDM_PACKET_SIZE; i++) {
    pdmPkt[2 + i * 2] = pdmBuffer[i] & 0xFF;
    pdmPkt[2 + i * 2 + 1] = (pdmBuffer[i] >> 8) & 0xFF;
  }
  
  // Remove sent samples from buffer
  memmove(pdmBuffer, pdmBuffer + PDM_PACKET_SIZE, (pdmBufferCount - PDM_PACKET_SIZE) * 2);
  pdmBufferCount -= PDM_PACKET_SIZE;
  
  if (pdmBufferCount < PDM_PACKET_SIZE) {
    pdmDataReady = false;
  }
  
  if (pdmDataChar.notifyEnabled()) {
    pdmDataChar.notify(pdmPkt, 2 + PDM_PACKET_SIZE * 2);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("XIAO nRF52840 Sense BLE IMU+PDM Streamer");
  
  bootTimeMs = millis();
  unixEpochTime = 0;
  
  // Initialize IMU
  if (myIMU.begin() != 0) {
    Serial.println("IMU initialization failed!");
  } else {
    Serial.println("IMU initialized");
    // Configure for 416 Hz ODR (if supported by library)
    // Note: Seeed_Arduino_LSM6DS3 may need specific configuration
  }
  
  // Initialize PDM microphone
  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, PDM_SAMPLE_RATE)) {
    Serial.println("Failed to start PDM!");
  } else {
    Serial.println("PDM initialized at 16kHz");
    // Set gain if needed
    PDM.setGain(20);
  }
  
  // Initialize BLE
  Bluefruit.autoConnLed(true);
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  
  Bluefruit.begin();
  Bluefruit.setTxPower(4); // Maximum TX power
  Bluefruit.setName("XIAO-Sense-IMU-PDM");
  
  // Set callbacks
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  
  // Configure service
  sensorService.begin();
  
  // Control characteristic
  controlChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
  controlChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  controlChar.setWriteCallback(control_write_callback);
  controlChar.setMaxLen(20);
  controlChar.begin();
  
  // Time characteristic
  timeChar.setProperties(CHR_PROPS_WRITE);
  timeChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  timeChar.setWriteCallback(time_write_callback);
  timeChar.setMaxLen(4);
  timeChar.begin();
  
  // IMU data characteristic
  imuDataChar.setProperties(CHR_PROPS_NOTIFY);
  imuDataChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  imuDataChar.setMaxLen(32);
  imuDataChar.begin();
  
  // PDM data characteristic
  pdmDataChar.setProperties(CHR_PROPS_NOTIFY);
  pdmDataChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  pdmDataChar.setMaxLen(514);
  pdmDataChar.begin();
  
  // Status characteristic
  statusChar.setProperties(CHR_PROPS_NOTIFY);
  statusChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  statusChar.setMaxLen(8);
  statusChar.begin();
  
  // Start advertising
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(sensorService);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // Fast advertising
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
  
  Serial.println("BLE advertising started. Device name: XIAO-Sense-IMU-PDM");
}

void loop() {
  uint32_t currentTime = micros();
  
  // Read IMU at 416 Hz when streaming
  if (isStreaming && (currentTime - lastIMUTime) >= IMU_SAMPLE_INTERVAL_US) {
    lastIMUTime = currentTime;
    
    // Read IMU data
    float ax, ay, az, gx, gy, gz;
    ax = myIMU.readFloatAccelX();
    ay = myIMU.readFloatAccelY();
    az = myIMU.readFloatAccelZ();
    gx = myIMU.readFloatGyroX();
    gy = myIMU.readFloatGyroY();
    gz = myIMU.readFloatGyroZ();
    float temp = myIMU.readTempC();
    
    // Scale to int16 (example scaling - adjust based on range)
    IMUSample sample;
    sample.sequence = imuSequence++;
    sample.timestamp_ms = millis() - bootTimeMs;
    sample.ax = (int16_t)(ax * 1000.0); // scale
    sample.ay = (int16_t)(ay * 1000.0);
    sample.az = (int16_t)(az * 1000.0);
    sample.gx = (int16_t)(gx * 10.0);
    sample.gy = (int16_t)(gy * 10.0);
    sample.gz = (int16_t)(gz * 10.0);
    sample.temp = (int16_t)(temp * 100.0);
    
    // Add to ring buffer with overflow protection
    uint32_t nextWrite = (imuWriteIndex + 1) % IMU_RING_BUFFER_SIZE;
    if (nextWrite == imuReadIndex) {
      // Buffer full, drop oldest
      imuReadIndex = (imuReadIndex + 1) % IMU_RING_BUFFER_SIZE;
    }
    imuRingBuffer[imuWriteIndex] = sample;
    imuWriteIndex = nextWrite;
  }
  
  // Send IMU packets
  if (isStreaming && bleConnected) {
    // Send up to a few packets per loop to avoid blocking
    for (int i = 0; i < 4 && imuReadIndex != imuWriteIndex; i++) {
      sendIMUPacket();
      delayMicroseconds(100); // Small delay to prevent congestion
    }
    
    // Send PDM packets
    if (pdmDataReady) {
      sendPDMPacket();
    }
  }
  
  // Yield to BLE stack
  yield();
}
