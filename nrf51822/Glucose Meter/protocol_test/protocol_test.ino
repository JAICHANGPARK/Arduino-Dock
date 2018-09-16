
#include <BLE_API.h>

#define DEVICE_NAME                       "BLE Serial"
#define TXRX_BUF_LEN                      20

BLE                                       ble;
Timeout                                   timeout;
Timeout                                   timeout1;
Ticker                                    ticker;

static uint8_t rx_buf[TXRX_BUF_LEN];
static uint8_t device_code_rx_buf[1];
static uint8_t rx_buf_num;
static uint8_t rx_state = 0;

// The uuid of service and characteristics
static const uint8_t service1_uuid[]        = {0x71, 0x3D, 0, 0, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_tx_uuid[]     = {0x71, 0x3D, 0, 3, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_rx_uuid[]     = {0x71, 0x3D, 0, 2, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_rx_uuid_2[]     = {0x71, 0x3D, 0x00, 0x04, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t uart_base_uuid_rev[]   = {0x1E, 0x94, 0x8D, 0xF1, 0x48, 0x31, 0x94, 0xBA, 0x75, 0x4C, 0x3E, 0x50, 0, 0, 0x3D, 0x71};

uint8_t tx_value[TXRX_BUF_LEN] = {0,};
uint8_t rx_value[TXRX_BUF_LEN] = {0,};

// Create characteristic and service
GattCharacteristic  characteristic1(service1_tx_uuid, tx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE );
GattCharacteristic  characteristic2(service1_rx_uuid, rx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
GattCharacteristic  characteristic3(service1_rx_uuid_2, rx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
GattCharacteristic *uartChars[] = {&characteristic1, &characteristic2, &characteristic3};
GattService         uartService(service1_uuid, uartChars, sizeof(uartChars) / sizeof(GattCharacteristic *));

uint8_t serialNumberReadCommand[7] = {0x8b, 0x11, 0x20, 0x13, 0x24, 0x10, 0x2a};
uint8_t savaDataCountGetCommand[7] = {0x8b, 0x11, 0x20, 0x18, 0x26, 0x10, 0x22};

boolean fisrtPhase = false;
boolean secondPhase = false;
boolean thirdPhase = false;

static uint8_t device_code_buf = 0;

void disconnectionCallBack(const Gap::DisconnectionCallbackParams_t *params) {
  ble.startAdvertising();
}

void gattServerWriteCallBack(const GattWriteCallbackParams *Handler) {
  uint8_t buf[TXRX_BUF_LEN];
  uint16_t bytesRead, index;

  if (Handler->handle == characteristic1.getValueAttribute().getHandle()) {
    ble.readCharacteristicValue(characteristic1.getValueAttribute().getHandle(), buf, &bytesRead);
    for (index = 0; index < bytesRead; index++) {
      Serial.write(buf[index]);
    }
  }
}

void m_uart_rx_handle() {
  //update characteristic data
  ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), rx_buf, rx_buf_num);
  ble.updateCharacteristicValue(characteristic3.getValueAttribute().getHandle(), device_code_rx_buf, 1);

  memset(rx_buf, 0x00, 20);
  rx_state = 0;
}

void uart_handle(uint32_t id, SerialIrq event) {
  // Serial rx IRQ
  if (event == RxIrq) {
    if (rx_state == 0) {
      rx_state = 1;
      timeout.attach_us(m_uart_rx_handle, 100000);
      rx_buf_num = 0;
    }
    while (Serial.available()) {
      if (rx_buf_num < 20) {
        rx_buf[rx_buf_num] = Serial.read();
        rx_buf_num++;
      }
      else {
        Serial.read();
      }
    }

    if (!fisrtPhase) {
      if (rx_buf[0] == 0x80 && rx_buf[1] == 0x10 && rx_buf[2] == 0x20) {
        fisrtPhase = true;
      }
    } else {
      if (!secondPhase) {
        if (rx_buf_num != 0) {
          device_code_rx_buf[0] = (((rx_buf[13] & 0x0f) << 4 ) | (rx_buf[14] & 0x0f));
          secondPhase = true;
        }
      } else {
        //        if (!thirdPhase) {
        //          if (rx_buf_num != 0) {
        //            device_code_rx_buf[0] = (((rx_buf[1] & 0x0f) << 4 ) | (rx_buf[2] & 0x0f));
        //            //thirdPhase = true;
        //          }
        //        }
      }
    }
  }
}

void wakeup_event_cb() {
  if (!fisrtPhase) {
    digitalWrite(D13, HIGH);
    Serial.write(0x80);
    //    fisrtPhase = true;
  }

  if (!secondPhase) {
    digitalWrite(D13, LOW);
    Serial.write(serialNumberReadCommand, 7);
    //    secondPhase = true;
  }

  if (!thirdPhase) {
    Serial.write(savaDataCountGetCommand, 7);
    //    thirdPhase = true;
  }
}

void setup() {
  // put your setup code here, to run once
  Serial.begin(9600);
  Serial.attach(uart_handle);
  ticker.attach(wakeup_event_cb, 2);

  ble.init();
  ble.onDisconnection(disconnectionCallBack);
  ble.onDataWritten(gattServerWriteCallBack);

  // setup adv_data and srp_data
  ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
  ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME, (const uint8_t *)"DiabetesGuru_BSM", sizeof("DiabetesGuru_BSM") - 1);
  ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS, (const uint8_t *)uart_base_uuid_rev, sizeof(uart_base_uuid_rev));
  // set adv_type
  ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
  // add service
  ble.addService(uartService);
  // set device name
  ble.setDeviceName((const uint8_t *)DEVICE_NAME);
  // set tx power,valid values are -40, -20, -16, -12, -8, -4, 0, 4
  ble.setTxPower(4);
  // set adv_interval, 100ms in multiples of 0.625ms.
  ble.setAdvertisingInterval(160);
  // set adv_timeout, in seconds
  ble.setAdvertisingTimeout(0);
  // start advertising
  ble.startAdvertising();

}

void loop() {
  // put your main code here, to run repeatedly:
  //  ble.waitForEvent();
  ble.waitForEvent();
}

