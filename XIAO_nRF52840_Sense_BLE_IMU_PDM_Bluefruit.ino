/*
 * XIAO nRF52840 Sense - BLE IMU + PDM Microphone Streamer (Bluefruit)
 * 
 * Board: Seeed nRF52 Boards > Seeed XIAO nRF52840 Sense
 * Libraries required:
 * - Seeed_Arduino_LSM6DS3 (for IMU)
 * - Seeed_Arduino_Mic (for PDM microphone)
 * - (Bluefruit is included with Seeed nRF52 Boards package)
 */

#include <Arduino.h>
#include <bluefruit.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include "PDM.h"

// BLE Service and Characteristics UUIDs
#define BLE_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_CONTROL_UUID       "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // Write/Notify
#define CHAR_TIME_UUID          "beb5483e-36e1-4688-b7f5-ea07361b26a9"  // Write
#define CHAR_IMU_DATA_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26aa"  // Notify
#define CHAR_PDM_DATA_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26ab"  // Notify
#define CHAR_STATUS_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26ac"  // Notify

// IMU Configuration
#define IMU_ODR_HZ              416
#define IMU_SAMPLE_INTERVAL_US  (1000000 / IMU_ODR_HZ)

// PDM Microphone Configuration
#define PDM_SAMPLE_RATE         16000
#define PDM_BUFFER_SIZE         1024
#define PDM_PACKET_SIZE         256

// Buffer sizes
#define IMU_RING_BUFFER_SIZE    1024

// Commands
#define CMD_START_STREAM        0x01
#define CMD_STOP_STREAM         0x02
#define CMD_GET_STATUS          0x03
#define CMD_SET_RECORDING       0x05

// Packet headers
#define PACKET_HEADER_IMU       0x49  // 'I'
#define PACKET_HEADER_PDM       0x50  // 'P'
#define PACKET_HEADER_STATUS    0x53  // 'S'

// IMU
LSM6DS3 myIMU(I2C_MODE, 0x6A);

// BLE
BLEService        sensorService(BLE_SERVICE_UUID);
BLECharacteristic controlChar(CHAR_CONTROL_UUID);
BLECharacteristic timeChar(CHAR_TIME_UUID);
BLECharacteristic imuDataChar(CHAR_IMU_DATA_UUID);
BLECharacteristic pdmDataChar(CHAR_PDM_DATA_UUID);
BLECharacteristic statusChar(CHAR_STATUS_UUID);

// PDM
static short pdmBuffer[PDM_BUFFER_SIZE];
static volatile int pdmBufferCount = 0;
static volatile bool pdmDataReady = false;

// IMU ring buffer
struct IMUSample {
  uint32_t sequence;
  uint32_t timestamp_ms;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t temp;
};

static IMUSample imuRingBuffer[IMU_RING_BUFFER_SIZE];
static volatile uint32_t imuWriteIndex = 0;
static volatile uint32_t imuReadIndex  = 0;
static volatile uint32_t imuSequence   = 0;

// State
static volatile bool isStreaming = false;
static volatile bool isRecording = false;
static volatile uint32_t bootTimeMs = 0;
static volatile uint32_t lastIMUTime = 0;
static volatile bool bleConnected = false;

// PDM callback
void onPDMdata() {
  int bytesAvailable = PDM.available();
  int samplesRead = bytesAvailable / 2;
  if (samplesRead <= 0) return;

  if (pdmBufferCount + samplesRead < PDM_BUFFER_SIZE) {
    PDM.read(pdmBuffer + pdmBufferCount, bytesAvailable);
    pdmBufferCount += samplesRead;
  } else {
    int overflow = (pdmBufferCount + samplesRead) - PDM_BUFFER_SIZE;
    if (overflow > 0 && overflow < pdmBufferCount) {
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

// BLE callbacks
void connect_callback(uint16_t conn_handle) {
  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  (void)conn;
  bleConnected = true;
  Serial.println("BLE Connected");
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;
  bleConnected = false;
  isStreaming  = false;
  isRecording  = false;
  Serial.println("BLE Disconnected");
}

void control_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  (void)conn_hdl; (void)chr;
  if (len < 1) return;
  uint8_t cmd = data[0];
  Serial.printf("Control cmd: 0x%02X\n", cmd);

  switch (cmd) {
    case CMD_START_STREAM:
      isStreaming = true;
      imuWriteIndex = 0; imuReadIndex = 0; imuSequence = 0;
      pdmBufferCount = 0; pdmDataReady = false;
      Serial.println("Streaming started");
      break;
    case CMD_STOP_STREAM:
      isStreaming = false;
      Serial.println("Streaming stopped");
      break;
    case CMD_GET_STATUS:
      {
        if (!bleConnected) break;
        uint8_t s[8];
        s[0]=PACKET_HEADER_STATUS; s[1]=bleConnected?1:0; s[2]=isStreaming?1:0; s[3]=isRecording?1:0;
        uint16_t q = (uint16_t)((imuWriteIndex >= imuReadIndex) ? (imuWriteIndex - imuReadIndex) : (IMU_RING_BUFFER_SIZE - (imuReadIndex - imuWriteIndex)));
        s[4]=q&0xFF; s[5]=(q>>8)&0xFF; s[6]=pdmBufferCount&0xFF; s[7]=(pdmBufferCount>>8)&0xFF;
        statusChar.notify(s,8);
      }
      break;
    case CMD_SET_RECORDING:
      if (len >= 2) {
        isRecording = (data[1] == 0x01);
        Serial.printf("Recording: %s\n", isRecording?"ON":"OFF");
      }
      break;
    default: break;
  }
}

void time_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  (void)conn_hdl; (void)chr;
  if (len >= 4) {
    uint32_t ts = data[0] | (data[1]<<8) | (data[2]<<16) | (data[3]<<24);
    if (ts < 1000000000UL) ts = ((uint32_t)data[0]<<24)|((uint32_t)data[1]<<16)|((uint32_t)data[2]<<8)|data[3];
    bootTimeMs = millis();
    (void)ts;
    Serial.println("Time sync received");
  }
}

void sendIMUPacket() {
  if (!bleConnected || !isStreaming) return;
  if (imuReadIndex == imuWriteIndex) return;
  IMUSample* sample = &imuRingBuffer[imuReadIndex];
  imuReadIndex = (imuReadIndex + 1) % IMU_RING_BUFFER_SIZE;

  uint8_t pkt[24];
  pkt[0]=PACKET_HEADER_IMU; pkt[1]=0x01;
  pkt[2]=sample->sequence&0xFF; pkt[3]=(sample->sequence>>8)&0xFF; pkt[4]=(sample->sequence>>16)&0xFF; pkt[5]=(sample->sequence>>24)&0xFF;
  pkt[6]=sample->timestamp_ms&0xFF; pkt[7]=(sample->timestamp_ms>>8)&0xFF; pkt[8]=(sample->timestamp_ms>>16)&0xFF; pkt[9]=(sample->timestamp_ms>>24)&0xFF;
  pkt[10]=sample->ax&0xFF; pkt[11]=(sample->ax>>8)&0xFF;
  pkt[12]=sample->ay&0xFF; pkt[13]=(sample->ay>>8)&0xFF;
  pkt[14]=sample->az&0xFF; pkt[15]=(sample->az>>8)&0xFF;
  pkt[16]=sample->gx&0xFF; pkt[17]=(sample->gx>>8)&0xFF;
  pkt[18]=sample->gy&0xFF; pkt[19]=(sample->gy>>8)&0xFF;
  pkt[20]=sample->gz&0xFF; pkt[21]=(sample->gz>>8)&0xFF;
  pkt[22]=sample->temp&0xFF; pkt[23]=(sample->temp>>8)&0xFF;

  if (imuDataChar.notifyEnabled()) {
    imuDataChar.notify(pkt, 24);
  }
}

void sendPDMPacket() {
  if (!bleConnected || !isStreaming) return;
  if (!pdmDataReady || pdmBufferCount < PDM_PACKET_SIZE) return;

  uint8_t pkt[2 + PDM_PACKET_SIZE*2];
  pkt[0]=PACKET_HEADER_PDM; pkt[1]=0x01;
  for (int i=0;i<PDM_PACKET_SIZE;i++) {
    pkt[2 + i*2]   = pdmBuffer[i] & 0xFF;
    pkt[2 + i*2+1] = (pdmBuffer[i] >> 8) & 0xFF;
  }
  memmove(pdmBuffer, pdmBuffer + PDM_PACKET_SIZE, (pdmBufferCount - PDM_PACKET_SIZE)*2);
  pdmBufferCount -= PDM_PACKET_SIZE;
  if (pdmBufferCount < PDM_PACKET_SIZE) pdmDataReady = false;

  if (pdmDataChar.notifyEnabled()) {
    pdmDataChar.notify(pkt, 2 + PDM_PACKET_SIZE*2);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("XIAO nRF52840 Sense BLE IMU+PDM Streamer (Bluefruit)");

  bootTimeMs = millis();

  if (myIMU.begin() != 0) Serial.println("IMU init failed");
  else Serial.println("IMU init ok");

  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, PDM_SAMPLE_RATE)) Serial.println("PDM init failed");
  else {
    Serial.println("PDM init ok @16kHz");
    PDM.setGain(20);
  }

  Bluefruit.autoConnLed(true);
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("XIAO-Sense-IMU-PDM");

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  sensorService.begin();

  controlChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
  controlChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  controlChar.setWriteCallback(control_write_callback);
  controlChar.setMaxLen(20);
  controlChar.begin();

  timeChar.setProperties(CHR_PROPS_WRITE);
  timeChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  timeChar.setWriteCallback(time_write_callback);
  timeChar.setMaxLen(4);
  timeChar.begin();

  imuDataChar.setProperties(CHR_PROPS_NOTIFY);
  imuDataChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  imuDataChar.setMaxLen(24);
  imuDataChar.begin();

  pdmDataChar.setProperties(CHR_PROPS_NOTIFY);
  pdmDataChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  pdmDataChar.setMaxLen(514);
  pdmDataChar.begin();

  statusChar.setProperties(CHR_PROPS_NOTIFY);
  statusChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  statusChar.setMaxLen(8);
  statusChar.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(sensorService);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32,244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);

  Serial.println("Advertising as XIAO-Sense-IMU-PDM");
}

void loop() {
  uint32_t now = micros();
  if (isStreaming && bleConnected && (now - lastIMUTime) >= IMU_SAMPLE_INTERVAL_US) {
    lastIMUTime = now;

    float ax = myIMU.readFloatAccelX();
    float ay = myIMU.readFloatAccelY();
    float az = myIMU.readFloatAccelZ();
    float gx = myIMU.readFloatGyroX();
    float gy = myIMU.readFloatGyroY();
    float gz = myIMU.readFloatGyroZ();
    float temp = myIMU.readTempC();

    IMUSample s;
    s.sequence = imuSequence++;
    s.timestamp_ms = millis() - bootTimeMs;
    s.ax = (int16_t)(ax*1000.0);
    s.ay = (int16_t)(ay*1000.0);
    s.az = (int16_t)(az*1000.0);
    s.gx = (int16_t)(gx*10.0);
    s.gy = (int16_t)(gy*10.0);
    s.gz = (int16_t)(gz*10.0);
    s.temp = (int16_t)(temp*100.0);

    uint32_t nw = (imuWriteIndex+1)%IMU_RING_BUFFER_SIZE;
    if (nw == imuReadIndex) {
      imuReadIndex = (imuReadIndex+1)%IMU_RING_BUFFER_SIZE;
    }
    imuRingBuffer[imuWriteIndex] = s;
    imuWriteIndex = nw;
  }

  if (isStreaming && bleConnected) {
    for (int i=0; i<3 && imuReadIndex != imuWriteIndex; i++) {
      sendIMUPacket();
      delayMicroseconds(50);
    }
    if (pdmDataReady) sendPDMPacket();
  }

  yield();
}