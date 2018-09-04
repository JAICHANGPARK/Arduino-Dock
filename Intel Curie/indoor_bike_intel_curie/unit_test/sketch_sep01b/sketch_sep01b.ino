#include <CurieBLE.h>

const int ledPin = 13; // set ledPin to use on-board LED

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // create service

// create switch characteristic and allow remote device to read and write
BLECharacteristic switchChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite, 20);
BLECharacteristic aChar("19B20001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite | BLENotify, 20);
BLECharacteristic bChar("19B30001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite | BLENotify, 20);
BLECharacteristic cChar("19B40001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite, 20);
BLECharacteristic dChar("19B50001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite, 20);
BLECharacteristic eChar("19B60001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite, 20);
void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT); // use the LED on pin 13 as an output

  // begin initialization
  BLE.begin();

  // set the local name peripheral advertises
  BLE.setLocalName("LEDCB");
  // set the UUID for the service this peripheral advertises
  BLE.setAdvertisedService(ledService);

  // add the characteristic to the service
  ledService.addCharacteristic(switchChar);
  ledService.addCharacteristic(aChar);
  ledService.addCharacteristic(bChar);
  ledService.addCharacteristic(cChar);
  ledService.addCharacteristic(dChar);
  ledService.addCharacteristic(eChar);
  // add service
  BLE.addService(ledService);

  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handlers for characteristic
  switchChar.setEventHandler(BLEWritten, switchCharacteristicWritten);
  aChar.setEventHandler(BLESubscribed, aCharacteristicWritten);
  bChar.setEventHandler(BLEUnsubscribed , bCharacteristicWritten);
  cChar.setEventHandler(BLEWritten, cCharacteristicWritten);
  dChar.setEventHandler(BLEWritten, dCharacteristicWritten);
  eChar.setEventHandler(BLEWritten, eCharacteristicWritten);
  // set an initial value for the characteristic
  switchChar.setValue(0, 0);

  // start advertising
  BLE.advertise();

  Serial.println(("Bluetooth device active, waiting for connections..."));
}

void loop() {
  // poll for BLE events
  BLE.poll();
}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

int rxHead = 0;
volatile static uint8_t rxBuffer[20];

void switchCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print("Characteristic event, written: ");
  size_t len = switchChar.valueLength();
  const unsigned char *data = switchChar.value();

  Serial.print("Got data:");
  Serial.write(data, len);

}
void aCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print("Characteristic event, written: aCharacteristicWritten ");
}

void bCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print("Characteristic event, written: bCharacteristicWritten ");
}

void cCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print("Characteristic event, written: cCharacteristicWritten ");
}

void dCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print("Characteristic event, written: dCharacteristicWritten ");
}

void eCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print("Characteristic event, written: eCharacteristicWritten ");
}


void addReceiveBytes(const uint8_t* bytes, size_t len) {
  // note increment rxHead befor writing
  // so need to increment rxTail befor reading
  for (size_t i = 0; i < len; i++) {
    rxHead = (rxHead + 1) % sizeof(rxBuffer);
    rxBuffer[rxHead] = bytes[i];
  }
}
