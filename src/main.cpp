#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <NimBLEDevice.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SSD1306Wire.h>
#include <map>
#include <string>

#include "ruuvilora_conf.h"

// Heltec WiFi LoRa 32 V2 pin definitions
constexpr uint8_t OLED_SDA = 4;
constexpr uint8_t OLED_SCL = 15;
constexpr uint8_t OLED_RST = 16;
constexpr uint8_t LORA_SCK = 5;
constexpr uint8_t LORA_MISO = 19;
constexpr uint8_t LORA_MOSI = 27;
constexpr uint8_t LORA_SS = 18;
constexpr uint8_t LORA_RST = 14;
constexpr uint8_t LORA_DIO0 = 26;
constexpr int8_t  LORA_TX_POWER = 14;  // dBm, stay within legal limits

// OLED display instance (128x64, I2C address 0x3c)
SSD1306Wire display(0x3c, OLED_SDA, OLED_SCL);

struct RuuviData {
  float temperature = NAN;
  float humidity = NAN;
  float pressure = NAN;
  float battery = NAN;
  int8_t rssi = 0;
  uint32_t lastSeen = 0;
};

std::map<String, RuuviData> ruuviTags;
NimBLEScan* bleScan = nullptr;

constexpr uint16_t RUUVI_COMPANY_ID = 0x0499;
constexpr uint8_t RUUVI_FORMAT_5 = 0x05;
constexpr uint8_t BLE_SCAN_DURATION_SECONDS = 5;

constexpr size_t LORA_MAX_PACKET_BYTES = 240;
constexpr size_t LORA_CHUNK_HEADER_BYTES = 4;
constexpr size_t LORA_MAX_CHUNK_DATA_BYTES = LORA_MAX_PACKET_BYTES - LORA_CHUNK_HEADER_BYTES;
constexpr uint16_t LORA_CHUNK_GAP_MS = 50;
static_assert(LORA_MAX_CHUNK_DATA_BYTES > 0, "LoRa chunk payload must be positive");

void sendRuuviDataOverLoRa();
void displayMessage(const char* line1, const char* line2 = nullptr, const char* line3 = nullptr);
uint16_t transmitPayloadChunks(const String& payload);

class RuuviAdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks {
 public:
  void onResult(NimBLEAdvertisedDevice* advertisedDevice) override {
    if (advertisedDevice == nullptr || !advertisedDevice->haveManufacturerData()) {
      return;
    }

    const std::string manufacturerData = advertisedDevice->getManufacturerData();
    if (manufacturerData.length() < 24) {
      return;
    }

    const uint8_t* data = reinterpret_cast<const uint8_t*>(manufacturerData.data());
    const uint16_t companyId = (static_cast<uint16_t>(data[1]) << 8) | data[0];
    
    if (companyId != RUUVI_COMPANY_ID || data[2] != RUUVI_FORMAT_5) {
      return;
    }

    Serial.printf("[BLE] Ruuvi packet from %s (RSSI %d, len %d)\n",
                  advertisedDevice->getAddress().toString().c_str(),
                  advertisedDevice->getRSSI(),
                  static_cast<int>(manufacturerData.length()));

    // Parse Ruuvi RAWv2 format
    RuuviData parsedData;
    const int16_t tempRaw = (static_cast<int16_t>(data[3]) << 8) | data[4];
    const uint16_t humidityRaw = (static_cast<uint16_t>(data[5]) << 8) | data[6];
    const uint16_t pressureRaw = (static_cast<uint16_t>(data[7]) << 8) | data[8];
    const uint16_t powerRaw = (static_cast<uint16_t>(data[15]) << 8) | data[16];

    parsedData.temperature = tempRaw * 0.005f;
    parsedData.humidity = humidityRaw * 0.0025f;
    parsedData.pressure = (pressureRaw + 50000) / 100.0f;
    const uint16_t batteryMv = powerRaw >> 5;
    parsedData.battery = batteryMv / 1000.0f;
    parsedData.rssi = advertisedDevice->getRSSI();
    parsedData.lastSeen = millis();

    const std::string macStd = advertisedDevice->getAddress().toString();
    ruuviTags[String(macStd.c_str())] = parsedData;
  }
};

RuuviAdvertisedDeviceCallbacks advertisedDeviceCallbacks;

void displayMessage(const char* line1, const char* line2, const char* line3) {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  if (line1) display.drawString(0, 0, line1);
  if (line2) display.drawString(0, 16, line2);
  if (line3) display.drawString(0, 32, line3);
  display.display();
}

void setupLoRa() {
  // Configure SPI pins for LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("[LoRa] Failed to initialize!");
    displayMessage("LoRa FAILED!", "Check wiring");
    while (1) {
      delay(1000);
    }
  }
  
  LoRa.setSpreadingFactor(LORA_SPREADING);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();
  LoRa.setTxPower(LORA_TX_POWER);
  
  Serial.println("[LoRa] Radio configured");
  Serial.printf("[LoRa] Frequency: %.1f MHz\n", LORA_FREQUENCY / 1E6);
  Serial.printf("[LoRa] Spreading Factor: %d\n", LORA_SPREADING);
  Serial.printf("[LoRa] Bandwidth: %.0f kHz\n", LORA_BW / 1E3);
  displayMessage("LoRa Ready", String("Freq: " + String(LORA_FREQUENCY / 1E6, 1) + " MHz").c_str());
  delay(1000);
}

void setupBLE() {
  NimBLEDevice::init("RuuviLoRaBridge");

  bleScan = NimBLEDevice::getScan();
  bleScan->setAdvertisedDeviceCallbacks(&advertisedDeviceCallbacks, true);
  bleScan->setInterval(160);
  bleScan->setWindow(160);
  bleScan->setActiveScan(true);
  bleScan->setDuplicateFilter(false);
  
  Serial.println("[BLE] Scanner configured");
  displayMessage("BLE Ready", "Scanning for", "Ruuvi tags...");
  delay(1000);
}


void setup() {
  // Initialize Serial
  Serial.begin(115200);
  delay(200);
  Serial.println("=================================");
  Serial.println("Ruuvi -> LoRa Bridge");
  Serial.println("=================================");
  
  // Reset OLED display
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(50);
  digitalWrite(OLED_RST, HIGH);
  
  // Initialize display
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  
  displayMessage("Ruuvi LoRa", "Bridge v1.0", "Initializing...");
  delay(2000);

  setupLoRa();
  setupBLE();
  
  displayMessage("System Ready", "Waiting for", "Ruuvi tags...");
  delay(1000);
}


void loop() {
  Serial.println("=================================");
  Serial.println("[BLE] Starting scan...");
  displayMessage("Scanning BLE...", String("Tags: " + String(ruuviTags.size())).c_str());
  
  if (bleScan != nullptr) {
    NimBLEScanResults results = bleScan->start(BLE_SCAN_DURATION_SECONDS, false);
    const int found = results.getCount();
    Serial.printf("[BLE] Scan complete. Found %d devices\n", found);
    bleScan->clearResults();
  }

  Serial.printf("[BLE] Total Ruuvi tags stored: %d\n", ruuviTags.size());
  displayMessage("Scan Complete", String("Tags: " + String(ruuviTags.size())).c_str(), "Sending LoRa...");

  sendRuuviDataOverLoRa();
  
  delay(5000);
}

uint16_t transmitPayloadChunks(const String& payload) {
  const size_t totalLen = payload.length();
  if (totalLen == 0) {
    Serial.println("[LoRa] Payload empty, skipping transmission");
    return 0;
  }

  const size_t requiredChunks = (totalLen + LORA_MAX_CHUNK_DATA_BYTES - 1) / LORA_MAX_CHUNK_DATA_BYTES;
  if (requiredChunks > UINT16_MAX) {
    Serial.println("[LoRa] Payload too large for 16-bit chunk headers");
    return 0;
  }

  const uint16_t chunkCount = static_cast<uint16_t>(requiredChunks);
  size_t offset = 0;
  uint16_t chunkIndex = 0;

  while (offset < totalLen) {
    size_t len = totalLen - offset;
    if (len > LORA_MAX_CHUNK_DATA_BYTES) {
      len = LORA_MAX_CHUNK_DATA_BYTES;
    }

    const String chunk = payload.substring(offset, offset + len);

    Serial.printf("[LoRa] Transmitting chunk %u/%u (%u bytes)\n",
                  static_cast<unsigned>(chunkIndex + 1),
                  static_cast<unsigned>(chunkCount),
                  static_cast<unsigned>(len));

    LoRa.beginPacket();
    LoRa.write(static_cast<uint8_t>((chunkCount >> 8) & 0xFF));
    LoRa.write(static_cast<uint8_t>(chunkCount & 0xFF));
    LoRa.write(static_cast<uint8_t>((chunkIndex >> 8) & 0xFF));
    LoRa.write(static_cast<uint8_t>(chunkIndex & 0xFF));
    LoRa.write(reinterpret_cast<const uint8_t*>(chunk.c_str()), len);
    LoRa.endPacket();

    offset += len;
    ++chunkIndex;
    delay(LORA_CHUNK_GAP_MS);
  }

  Serial.printf("[LoRa] Completed transmission: %u bytes across %u chunks\n",
                static_cast<unsigned>(totalLen),
                static_cast<unsigned>(chunkCount));
  return chunkCount;
}

void sendRuuviDataOverLoRa() {
  if (ruuviTags.empty()) {
    Serial.println("[LoRa] No Ruuvi data collected yet");
    displayMessage("No tags found", "Waiting...");
    return;
  }

  const size_t tagCount = ruuviTags.size();
  JsonDocument doc;

  doc["count"] = tagCount;
  doc["ts"] = millis();
  JsonArray tags = doc["tags"].to<JsonArray>();

  const uint32_t now = millis();
  
  Serial.println("[LoRa] Preparing data for transmission:");
  for (const auto& entry : ruuviTags) {
    const RuuviData& data = entry.second;
    JsonObject jsonTag = tags.add<JsonObject>();
    jsonTag["id"] = entry.first;
    jsonTag["t"] = serialized(String(data.temperature, 2));
    jsonTag["h"] = serialized(String(data.humidity, 2));
    jsonTag["p"] = serialized(String(data.pressure, 2));
    jsonTag["bat"] = serialized(String(data.battery, 3));
    jsonTag["rssi"] = data.rssi;
    jsonTag["age"] = static_cast<uint16_t>((now - data.lastSeen) / 1000);
    
    Serial.printf("  %s: T=%.2f°C H=%.2f%% P=%.2fhPa Bat=%.3fV RSSI=%ddBm\n",
                  entry.first.c_str(), data.temperature, data.humidity, 
                  data.pressure, data.battery, data.rssi);
  }

  String payload;
  serializeJson(doc, payload);

  const uint16_t chunkCount = transmitPayloadChunks(payload);
  if (chunkCount == 0) {
    displayMessage("TX Skipped", "Payload issue");
    return;
  }

  Serial.printf("[LoRa] Sent %u bytes for %u tags (%u chunks)\n",
                static_cast<unsigned>(payload.length()),
                static_cast<unsigned>(tagCount),
                static_cast<unsigned>(chunkCount));
  
  const String statsLine = String(payload.length()) + "B / " + String(chunkCount) + " pkt";
  displayMessage("TX Complete", 
                 String("Tags: " + String(tagCount)).c_str(),
                 statsLine.c_str());
}