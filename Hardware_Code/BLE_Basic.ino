#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/* ─── UUIDs ─────────────────────────────────────────────── */
#define SERVICE_UUID      "a32be81d-570e-4ad9-bf2a-64fdfe3db515"
#define WRITE_CHAR_UUID   "c6b0278a-f9b5-4306-8eee-5d74d746bcc3"
#define READ_CHAR_UUID    "f53d0832-1177-479b-8046-1b76534390cb"
#define NOTIFY_CHAR_UUID  "b3af0550-1ba9-4efb-aac7-14287a527e06"

/* ─── BLE objects ────────────────────────────────────────── */
BLEServer         *pServer     = nullptr;
BLECharacteristic *writeChar   = nullptr;
BLECharacteristic *readChar    = nullptr;
BLECharacteristic *notifyChar  = nullptr;

/* ─── Connection state ───────────────────────────────────── */
volatile bool deviceConnected     = false;
volatile bool previouslyConnected = false; // ← key flag for loop() restart logic
bool          timeSynced          = false;

/* ─── Time sync ──────────────────────────────────────────── */
uint32_t baseEpoch  = 0;
uint32_t baseMillis = 0;

/* ─── Handshake timer ────────────────────────────────────── */
unsigned long connectTime = 0;
const unsigned long HANDSHAKE_TIMEOUT = 5000;

int counter = 0;

/* ════════════════════════════════════════════════════════════
   WRITE CALLBACK
   ════════════════════════════════════════════════════════════ */
class TimeWriteCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    String value = pChar->getValue();
    if (value.length() == 0) return;

    uint32_t receivedTime = (uint32_t)value.toInt();

    if (receivedTime < 1600000000UL) {
      Serial.println("[BLE] Invalid timestamp — ignored");
      return;
    }

    baseEpoch  = receivedTime;
    baseMillis = millis();
    timeSynced = true;

    Serial.print("[BLE] Timestamp received: ");
    Serial.println(baseEpoch);
    Serial.println("[BLE] Handshake successful!");
  }
};

/* ════════════════════════════════════════════════════════════
   SERVER CALLBACKS
   Only set flags here — never call advertising->start()
   inside a BLE callback on ESP32 Core 3.x (stack-unsafe).
   ════════════════════════════════════════════════════════════ */
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    deviceConnected     = true;
    previouslyConnected = false;       // clear so loop won't restart ads
    timeSynced          = false;
    connectTime         = millis();
    Serial.println("[BLE] Client connected — awaiting timestamp...");

    // Optional: reduce connection interval for faster throughput
    // server->updateConnParams(server->getConnId(), 6, 6, 0, 400);
  }

  void onDisconnect(BLEServer *server) override {
    // ── Only set flags. Do NOT call advertising->start() here. ──
    deviceConnected     = false;
    previouslyConnected = true;        // ← signals loop() to restart ads
    timeSynced          = false;
    Serial.println("[BLE] Client disconnected");
  }
};

/* ════════════════════════════════════════════════════════════
   HELPER — restart advertising safely from loop() context
   ════════════════════════════════════════════════════════════ */
void restartAdvertising() {
  // Small delay lets the BLE stack finish its disconnect cleanup
  delay(200);

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);   // helps iOS connectivity
  pAdvertising->setMaxPreferred(0x12);

  BLEDevice::startAdvertising();         // Core 3.x: use BLEDevice::startAdvertising()
  Serial.println("[BLE] Advertising restarted — ready for reconnection");
}

/* ════════════════════════════════════════════════════════════
   SETUP
   ════════════════════════════════════════════════════════════ */
void setup() {
  Serial.begin(115200);
  Serial.println("[SYS] Booting...");

  BLEDevice::init("ESP32_BasicBLE");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *service = pServer->createService(SERVICE_UUID);

  /* WRITE — timestamp / command channel */
  writeChar = service->createCharacteristic(
                WRITE_CHAR_UUID,
                BLECharacteristic::PROPERTY_WRITE
              );
  writeChar->setCallbacks(new TimeWriteCallback());

  /* READ — static status string */
  readChar = service->createCharacteristic(
               READ_CHAR_UUID,
               BLECharacteristic::PROPERTY_READ
             );
  readChar->setValue("ESP32 Ready");

  /* NOTIFY — sensor data channel */
  notifyChar = service->createCharacteristic(
                 NOTIFY_CHAR_UUID,
                 BLECharacteristic::PROPERTY_NOTIFY
               );
  notifyChar->addDescriptor(new BLE2902());

  service->start();

  /* Initial advertising */
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising started...");
}

/* ════════════════════════════════════════════════════════════
   LOOP
   ════════════════════════════════════════════════════════════ */
void loop() {

  /* ── RECONNECT HANDLER ───────────────────────────────────
     Runs once after each disconnect, safely in loop() context.
     previouslyConnected is set by onDisconnect() callback.    */
  if (!deviceConnected && previouslyConnected) {
    previouslyConnected = false;   // consume the flag first
    restartAdvertising();
    return;
  }

  /* ── HANDSHAKE TIMEOUT ───────────────────────────────────
     If phone connected but never sent timestamp within window,
     force-disconnect and fall back to advertising.            */
  if (deviceConnected && !timeSynced) {
    if (millis() - connectTime > HANDSHAKE_TIMEOUT) {
      Serial.println("[BLE] Handshake timeout — disconnecting client");
      pServer->disconnect(pServer->getConnId());
      // onDisconnect() will fire → previouslyConnected = true → restartAdvertising()
    }
    return; // Nothing else to do until handshake completes
  }

  /* ── NORMAL OPERATION (connected + synced) ───────────────
     Guard every notify with deviceConnected check to avoid
     writing to the stack after disconnect begins.            */
  if (deviceConnected && timeSynced) {
    counter++;

    String msg = "Count: " + String(counter);

    // Safe notify: check connection flag immediately before sending
    if (deviceConnected) {
      notifyChar->setValue(msg.c_str());
      notifyChar->notify();
      Serial.println("[DATA] " + msg);
    }

    delay(1000); // 1 Hz — replace with non-blocking millis() for production
  }
}