#include <HX711.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include <BLE2902.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define pinDT 22
#define pinSCK 21

#define valueToMilliV128 1.442e-6
#define valueToMilliV64 2.91109E-06

#define scaleServiceUUID BLEUUID("12c3916f-6768-40b8-a394-0ce8654c34be")
#define scaleCharacteristicUUID BLEUUID("53d33a98-988e-4a98-812a-48da102f9c5b")
#define readModeCharacteristicUUID BLEUUID("96d56383-b5b7-4c1f-8265-cad11350c802")
#define scaleCommandCharacteristicUUID BLEUUID("86d822f0-19ad-46f5-a158-d2bb5ad4d089")

#define READ_SCALE_ONCE 0x01
#define READ_SCALE_AUTOMATICALLY 0x02
#define RESTART_MEASUREMENTS 0x03

const String sgdPath = "/strain_gauge_data";
String newFileName;

HX711 scale;

BLEServer *pServer = NULL;
BLECharacteristic *pScaleCharacteristic = NULL;
// contains value of 0x00 and 0x01 only
BLECharacteristic *pAutoModeCharacteristic = NULL;
BLECharacteristic *pScaleCommandCharacteristic = NULL;

void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.path(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char *path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

bool directoryExists(String dirPath) {
  return SD.open(dirPath.c_str());
}

void createSgdDirectoryIfAbsent() {
  if (!directoryExists(sgdPath)) {
    createDir(SD, sgdPath.c_str());
  }
}

void updateReadModeCharacteristic(bool newValue) {
  uint8_t data[] = { (uint8_t)newValue };
  pAutoModeCharacteristic->setValue(data, 1);
  if (deviceIsConnected()) pAutoModeCharacteristic->notify();
}

void createNewFileAndWriteHeader() {
  uint64_t newFileNumber = 1;

  File sgdDirectory = SD.open(sgdPath.c_str());

  while (sgdDirectory.openNextFile()) {
    newFileNumber++;
  }

  newFileName = "/l_" + String(newFileNumber) + ".txt";
  writeFile(SD, (sgdPath + newFileName).c_str(), "timeInMillis,scaleValue\n");
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    Serial.println("Device connected");
  };

  void onDisconnect(BLEServer *pServer) {
    Serial.println("Device disconnected");

    onDeviceDisconnect();
    pServer->startAdvertising();
  }
};

class CharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t *param) {
    if (pCharacteristic->getUUID().equals(scaleCommandCharacteristicUUID)) {
      uint8_t command = pCharacteristic->getData()[0];

      switch (command) {
        case READ_SCALE_ONCE:
          {
            readScaleOnce();
          }
          break;
        case READ_SCALE_AUTOMATICALLY:
          {
            bool value = pCharacteristic->getData()[1];
            updateReadModeCharacteristic(value);
          }
          break;
        case RESTART_MEASUREMENTS:
          {
            updateReadModeCharacteristic(false);
            createNewFileAndWriteHeader();
          }
          break;
        default:
          {
            Serial.println("Unkown command");
          }
          break;
      }
    }
  }

  void onRead(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t *param) {
  }

  void onNotify(BLECharacteristic *pCharacteristic) {
  }
};

bool deviceIsConnected() {
  return pServer->getConnectedCount() == 1;
}

void onDeviceDisconnect() {
  updateReadModeCharacteristic(false);

  BLEDescriptor *pScaleDescriptor2902 = pScaleCharacteristic->getDescriptorByUUID(
    (new BLE2902())->getUUID());
  BLEDescriptor *pAutoModeDescriptor2902 = pAutoModeCharacteristic->getDescriptorByUUID(
    (new BLE2902())->getUUID());

  uint8_t data[] = { 0x00, 0x00 };

  pScaleDescriptor2902->setValue(data, 2);
  pAutoModeDescriptor2902->setValue(data, 2);
}

struct ScaleMeasurement {
  int16_t value;
  uint32_t timeMillisecond32;
} __attribute__((packed));

void readScaleOnce() {
  double result = scale.get_value();
  double resultMilliV = result * valueToMilliV64;

  ScaleMeasurement measurement = {
    int16_t(resultMilliV * 1e3),
    millis(),
  };

  Serial.print(measurement.timeMillisecond32);
  Serial.print(",");
  Serial.println(resultMilliV, 5);

  // Update BLE char value, but notify only if device is connected
  pScaleCharacteristic->setValue((uint8_t *)&measurement, sizeof(measurement));
  if (deviceIsConnected()) pScaleCharacteristic->notify();

  String data = String(measurement.timeMillisecond32) + "," + String(resultMilliV, (uint)5) + "\n";
  appendFile(SD, (sgdPath + newFileName).c_str(), data.c_str());
}

void setup() {
  // Start serial
  Serial.begin(115200);

  // Start scale
  scale.begin(pinDT, pinSCK, 64);

  // Start SD card
  SD.begin();
  createSgdDirectoryIfAbsent();
  createNewFileAndWriteHeader();

  // Create the BLE Device
  BLEDevice::init("ESP Scale");

  // Create server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pScaleService = pServer->createService(scaleServiceUUID);

  pScaleCharacteristic = pScaleService->createCharacteristic(
    scaleCharacteristicUUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);

  pScaleCharacteristic->addDescriptor(new BLE2902());
  pScaleCharacteristic->setCallbacks(new CharacteristicCallbacks());

  pAutoModeCharacteristic = pScaleService->createCharacteristic(
    readModeCharacteristicUUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);

  pAutoModeCharacteristic->addDescriptor(new BLE2902());
  pAutoModeCharacteristic->setCallbacks(new CharacteristicCallbacks());

  pScaleCommandCharacteristic = pScaleService->createCharacteristic(
    scaleCommandCharacteristicUUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  pScaleCommandCharacteristic->setCallbacks(new CharacteristicCallbacks());

  pScaleService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(scaleServiceUUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);

  BLEDevice::startAdvertising();

  // Set initial value to read mode
  updateReadModeCharacteristic(false);
}

void loop() {
  bool readScaleAutomatically = pAutoModeCharacteristic->getData()[0];
  if (readScaleAutomatically) readScaleOnce();
}
