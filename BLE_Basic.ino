#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/* UUIDs */
#define SERVICE_UUID        "a32be81d-570e-4ad9-bf2a-64fdfe3db515"
#define WRITE_CHAR_UUID    "c6b0278a-f9b5-4306-8eee-5d74d746bcc3"
#define READ_CHAR_UUID     "f53d0832-1177-479b-8046-1b76534390cb"
#define NOTIFY_CHAR_UUID   "b3af0550-1ba9-4efb-aac7-14287a527e06"

/* BLE Objects */
BLEServer *pServer;
BLECharacteristic *writeChar;
BLECharacteristic *readChar;
BLECharacteristic *notifyChar;

/* State variables */
bool deviceConnected = false;
bool timeSynced = false;

/* Time sync variables */
uint32_t baseEpoch = 0;
uint32_t baseMillis = 0;

/* Handshake timer */
unsigned long connectTime = 0;
const unsigned long HANDSHAKE_TIMEOUT = 5000; // 5 seconds

int counter = 0;

/* ================= WRITE CALLBACK ================= */
class TimeWriteCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) {

    String value = pChar->getValue();   // Arduino String

    if (value.length() == 0) return;

    uint32_t receivedTime = value.toInt();

    // Sanity check (>= year 2020)
    if (receivedTime < 1600000000) {
      Serial.println("Invalid timestamp received");
      return;
    }

    baseEpoch = receivedTime;
    baseMillis = millis();
    timeSynced = true;

    Serial.print("Timestamp received: ");
    Serial.println(baseEpoch);
    Serial.println("Handshake successful!");
  }
};

/* ================= SERVER CALLBACKS ================= */
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) {
    deviceConnected = true;
    timeSynced = false;
    connectTime = millis();
    Serial.println("Client connected, waiting for timestamp...");
  }

  void onDisconnect(BLEServer* server) {
    deviceConnected = false;
    timeSynced = false;
    Serial.println("Client disconnected");
    server->getAdvertising()->start();
  }
};

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);

  BLEDevice::init("ESP32_BasicBLE");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *service = pServer->createService(SERVICE_UUID);

  // WRITE characteristic (timestamp)
  writeChar = service->createCharacteristic(
                WRITE_CHAR_UUID,
                BLECharacteristic::PROPERTY_WRITE
              );
  writeChar->setCallbacks(new TimeWriteCallback());

  // READ characteristic
  readChar = service->createCharacteristic(
               READ_CHAR_UUID,
               BLECharacteristic::PROPERTY_READ
             );
  readChar->setValue("ESP32 Ready");

  // NOTIFY characteristic
  notifyChar = service->createCharacteristic(
                 NOTIFY_CHAR_UUID,
                 BLECharacteristic::PROPERTY_NOTIFY
               );
  notifyChar->addDescriptor(new BLE2902());

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->start();

  Serial.println("BLE Advertising Started...");
}

/* ================= LOOP ================= */
void loop() {

  /* ---- HANDSHAKE TIMEOUT ---- */
  if (deviceConnected && !timeSynced) {
    if (millis() - connectTime > HANDSHAKE_TIMEOUT) {
      Serial.println("Handshake failed. Disconnecting client...");
      pServer->disconnect(0);
      delay(100);
    }
    return; // Do nothing else until handshake
  }

  /* ---- AFTER HANDSHAKE SUCCESS ---- */
  if (deviceConnected ) {      //&& timeSynced
    counter++;

    String msg = "Count: " + String(counter);

    notifyChar->setValue(msg.c_str());
    notifyChar->notify();

    Serial.println(msg);

    delay(1000); // 1 Hz
  }
}
