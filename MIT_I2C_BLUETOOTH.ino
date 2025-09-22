#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// I2C addresses to probe
const uint8_t ADDRS[] = {0x4D, 0x48, 0x4F};

// BLE service/characteristic UUIDs (random example UUIDs, replace if needed)
#define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

// ---------- I2C Helpers ----------
bool i2cRead16(uint8_t a, uint8_t r, uint16_t &v){
  Wire.beginTransmission(a);
  Wire.write(r);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)a, 2) != 2) return false;
  v = (Wire.read() << 8) | Wire.read();
  return true;
}

bool i2cWrite16(uint8_t a, uint8_t r, uint16_t v){
  Wire.beginTransmission(a);
  Wire.write(r);
  Wire.write((uint8_t)(v >> 8));
  Wire.write((uint8_t)(v & 0xFF));
  return Wire.endTransmission() == 0;
}

// Masks
static const uint16_t RES_MASK   = 0x0060; 
static const uint16_t SHDN_MASK  = 0x0100; 
static const uint16_t RATE_MASK  = 0x0006; 
static const uint16_t RATE_1SPS  = 0x0002; 

bool force12_and_run(uint8_t a){
  uint16_t cfg_before;
  if(!i2cRead16(a, 0x01, cfg_before)) return false;
  uint16_t cfg = cfg_before;
  cfg &= ~RES_MASK; cfg |= RES_MASK;  // 12-bit
  cfg &= ~SHDN_MASK;                  // continuous
  cfg = (cfg & ~RATE_MASK) | RATE_1SPS;
  if(!i2cWrite16(a, 0x01, cfg)) return false;
  delay(150);
  uint16_t cfg_after;
  if(!i2cRead16(a, 0x01, cfg_after)) return false;
  return (((cfg_after >> 5) & 0x3) == 0b11);
}

bool read12(uint8_t a, float &tC){
  Wire.beginTransmission(a);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)a, 2) != 2) return false;
  uint16_t w  = (Wire.read() << 8) | Wire.read();
  int16_t raw = ((int16_t)w) >> 4;
  if (raw & 0x0800) raw |= 0xF000;
  tC = raw * 0.0625f;
  return true;
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Init I2C sensors
  for (uint8_t a : ADDRS){
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      force12_and_run(a);
    }
  }

  // Init BLE
  BLEDevice::init("ESP32-TempFiber");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("BLE service started, waiting for nRF Connect...");
}

// ---------- Loop ----------
void loop() {
  String payload = "";  // will hold all temps

  for (uint8_t a : ADDRS){
    Wire.beginTransmission(a);
    if (Wire.endTransmission() != 0) continue;

    float tC;
    if (read12(a, tC)){
      payload += "Addr 0x";
      if(a<0x10) payload += "0";
      payload += String(a, HEX);
      payload += ": ";
      payload += String(tC, 2);
      payload += " C  |  ";
    }
  }

  if (payload.length() > 0) {
    Serial.println(payload);
    pCharacteristic->setValue(payload.c_str());
    pCharacteristic->notify();  // send to nRF Connect
  }

  delay(1000); // 1 Hz updates
}