/*
   Copyright (c) 2016 RedBear

   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.
*/
#include <nRF5x_BLE_API.h>

#define DEVICE_NAME                       "BLE Serial"
#define TXRX_BUF_LEN                      20

BLE                                       ble;
Timeout                                   timeout, timeout1, timeout2;
Ticker                                    ticker;

static uint8_t rx_buf[TXRX_BUF_LEN];
static uint8_t rx_buf_num;
static uint8_t rx_state = 0;


// The uuid of service and characteristics
static const uint8_t service1_uuid[]        = {0x71, 0x3D, 0x00, 0, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_tx_uuid[]     = {0x71, 0x3D, 0x00, 1, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_rx_uuid[]     = {0x71, 0x3D, 0x00, 2, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_rx_uuid_2[]   = {0x71, 0x3D, 0x00, 0x03, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_uuid_0[]   = {0x71, 0x3D, 0x00, 0x04, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_uuid_1[]   = {0x71, 0x3D, 0x00, 0x05, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_uuid_2[]   = {0x71, 0x3D, 0x00, 0x06, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};

static const uint8_t uart_base_uuid_rev[]   = {0x1E, 0x94, 0x8D, 0xF1, 0x48, 0x31, 0x94, 0xBA, 0x75, 0x4C, 0x3E, 0x50, 0, 0, 0x3D, 0x71};


static const uint8_t service2_uuid[]        = {0x71, 0x3D, 0x00, 0x01, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service2_tx_uuid[]     = {0x71, 0x3D, 0x00, 1, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};

uint8_t tx_value[TXRX_BUF_LEN] = {0,};
uint8_t rx_value[TXRX_BUF_LEN] = {0,};

// Create characteristic and service
GattCharacteristic  characteristic1(service1_tx_uuid, tx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE );
GattCharacteristic  characteristic2(service1_rx_uuid, rx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
GattCharacteristic  characteristic3(service1_rx_uuid_2, rx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
GattCharacteristic  characteristic4(service1_uuid_0, rx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ );
GattCharacteristic  characteristic5(service1_uuid_1, rx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ );
GattCharacteristic  characteristic6(service1_uuid_2, rx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ );

GattCharacteristic *uartChars[] = {&characteristic1, &characteristic4, &characteristic5, &characteristic6, &characteristic2, &characteristic3};
GattService         uartService(service1_uuid, uartChars, sizeof(uartChars) / sizeof(GattCharacteristic *));

GattCharacteristic  characteristic7(service2_tx_uuid, rx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ );
GattCharacteristic  *uartChars2[] = {&characteristic7};
GattService         uartService2(service2_uuid, uartChars2, sizeof(uartChars2) / sizeof(GattCharacteristic *));

uint8_t echo[1] = {0x80};
uint8_t serialNumberReadCommand[7] = {0x8b, 0x11, 0x20, 0x13, 0x24, 0x10, 0x2a};
uint8_t savaDataCountGetCommand[7] = {0x8b, 0x11, 0x20, 0x18, 0x26, 0x10, 0x22};

uint8_t getDataCommand[7] = {0x8b, 0x1d, 0x22, 0x10, 0x20, 0x10, 0x28};

boolean fisrtPhase = false;
boolean secondPhase = false;
boolean thirdPhase = false;
boolean fourPhase = false;

static uint8_t device_code_buf = 0;
uint8_t saveCount = 0;
int indexCount = 0;

static uint8_t device_code_rx_buf[1];
static uint8_t save_data_rx_buf[1];
static uint8_t data_rx_buf[1];


void disconnectionCallBack(const Gap::DisconnectionCallbackParams_t *params) {
  ble.startAdvertising();
}

void gattServerWriteCallBack(const GattWriteCallbackParams *Handler) {
  uint16_t index;
  //  if (Handler->handle == characteristic1.getValueAttribute().getHandle()) {
  //    for (index = 0; index < Handler->len; index++) {
  //      Serial.write(Handler->data[index]);
  //    }
  //  }
  if (Handler->handle == characteristic4.getValueAttribute().getHandle()) {
    //    for (index = 0; index < Handler->len; index++) {
    //      Serial.write(Handler->data[index]);
    //    }

    digitalWrite(D13, HIGH);
    Serial.write(0x80);

  }

  if (Handler->handle == characteristic5.getValueAttribute().getHandle()) {
    //    for (index = 0; index < Handler->len; index++) {
    //      Serial.write(Handler->data[index]);
    //    }

    digitalWrite(D13, LOW);
    Serial.write(serialNumberReadCommand, 7);
    //    secondPhase = true;

  }

  if (Handler->handle == characteristic6.getValueAttribute().getHandle()) {
    Serial.write(savaDataCountGetCommand, 7);
  }

  if (Handler->handle == characteristic7.getValueAttribute().getHandle()) {
    if (fisrtPhase && secondPhase && thirdPhase) {
      Serial.write(getDataCommand, 7);
    }
  }
}

void m_uart_rx_handle() {
  //update characteristic data
  ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), rx_buf, rx_buf_num);
  memset(rx_buf, 0x00, 20);
  rx_state = 0;
}

void d_uart_rx_handle() {
  //update characteristic data
  ble.updateCharacteristicValue(characteristic3.getValueAttribute().getHandle(), device_code_rx_buf, 1);
  memset(device_code_rx_buf, 0x00, 1);
}

void c_uart_rx_handle() {
  //update characteristic data
  ble.updateCharacteristicValue(characteristic3.getValueAttribute().getHandle(), save_data_rx_buf, 1);
  memset(save_data_rx_buf, 0x00, 1);
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
      } else {
        Serial.read();
      }
    }

    if (!fisrtPhase) { // 첫번째 프로토콜 시작
      if (rx_buf[0] == 0x80 && rx_buf[1] == 0x10 && rx_buf[2] == 0x20) { // 올바른 값이 들어온다면
        data_rx_buf[0] = 0x10;
        fisrtPhase = true;
        ble.updateCharacteristicValue(characteristic4.getValueAttribute().getHandle(), data_rx_buf, 1);
        return ;
      }  
    } else { // 첫번째 프로토콜이 검증됬다면
      if (!secondPhase) { // 두번째 프로토콜 시작
        device_code_rx_buf[0] = (((rx_buf[13] & 0x0f) << 4 ) | (rx_buf[14] & 0x0f));
        timeout1.attach_us(d_uart_rx_handle, 100000);
        ble.updateCharacteristicValue(characteristic5.getValueAttribute().getHandle(), device_code_rx_buf, 1);
        if (device_code_rx_buf[0] == 0x02) {
          secondPhase = true;
          return ;
        }
      }
      else {
        if (!thirdPhase) {
          save_data_rx_buf[0] = (((rx_buf[1] & 0x0f) << 4 ) | (rx_buf[2] & 0x0f));
          saveCount = save_data_rx_buf[0];
          //          timeout2.attach_us(c_uart_rx_handle, 100000);
          ble.updateCharacteristicValue(characteristic6.getValueAttribute().getHandle(), save_data_rx_buf, 1);
          thirdPhase = true;
          return ;
        }
      }
    }
    if (fisrtPhase && secondPhase && thirdPhase) {
      data_rx_buf[0] = rx_buf_num;
      ble.updateCharacteristicValue(characteristic7.getValueAttribute().getHandle(), save_data_rx_buf, 1);
    }

  }
}

void setup() {
  // put your setup code here, to run once
  Serial.begin(9600);
  Serial.attach(uart_handle);
  //ticker.attach(&flip, 1);

  ble.init();
  ble.onDisconnection(disconnectionCallBack);
  ble.onDataWritten(gattServerWriteCallBack);

  // setup adv_data and srp_data
  ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
  ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME, (const uint8_t *)"DiabetesGuruBSM", sizeof("DiabetesGuruBSM") - 1);
  ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS, (const uint8_t *)uart_base_uuid_rev, sizeof(uart_base_uuid_rev));
  // set adv_type
  ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
  // add service
  ble.addService(uartService);
  ble.addService(uartService2);

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
  ble.waitForEvent();

  //
  //  digitalWrite(D13, HIGH);
  //  Serial.write(0x80);
  //  delay(2000);
  //  digitalWrite(D13, LOW);
  //  Serial.write(serialNumberReadCommand, 7);
  //  delay(2000);
  //  Serial.write(savaDataCountGetCommand, 7);
  //  delay(2000);
  //  Serial.write(getDataCommand, 7);

}
