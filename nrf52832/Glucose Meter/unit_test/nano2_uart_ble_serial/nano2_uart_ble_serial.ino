
#include <nRF5x_BLE_API.h>

#define DEVICE_NAME                       "BLE Serial"
#define TXRX_BUF_LEN                      20
#define PROTOCOL_ONE_FIRTS_PHASE_LENGTH   3
#define PROTOCOL_ONE_SECOND_PHASE_LENGTH  30
#define PROTOCOL_ONE_THIRD_PHASE_LENGTH   6
#define PROTOCOL_ONE_FORTH_PHASE_LENGTH   24

#define GLUCOSE_DATA_SIZE                 8

BLE                                       ble;
Timeout                                   timeout, timeout1, timeout2;
Ticker                                    ticker;

static uint8_t rx_buf[30];
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
GattCharacteristic  characteristic4(service1_uuid_0, rx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY );
GattCharacteristic  characteristic5(service1_uuid_1, rx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY );
GattCharacteristic  characteristic6(service1_uuid_2, rx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY );

GattCharacteristic *uartChars[] = {&characteristic1, &characteristic4, &characteristic5, &characteristic6, &characteristic2, &characteristic3};
GattService         uartService(service1_uuid, uartChars, sizeof(uartChars) / sizeof(GattCharacteristic *));

GattCharacteristic  characteristic7(service2_tx_uuid, rx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ );
GattCharacteristic  *uartChars2[] = {&characteristic7};
GattService         uartService2(service2_uuid, uartChars2, sizeof(uartChars2) / sizeof(GattCharacteristic *));

uint8_t echo[1] = {0x80}; //echo
uint8_t serialNumberReadCommand[7] = {0x8b, 0x11, 0x20, 0x13, 0x24, 0x10, 0x2a}; // 0x103b
uint8_t savaDataCountGetCommand[7] = {0x8b, 0x11, 0x20, 0x18, 0x26, 0x10, 0x22}; // 0x1086

uint8_t getDataCommand[7] = {0x8b, 0x1d, 0x22, 0x10, 0x20, 0x10, 0x28};

volatile boolean protocolOneFlag = false;
volatile boolean protocolThreeFlag = false;

volatile boolean fisrtPhase = false;
volatile boolean secondPhase = false;
volatile boolean thirdPhase = false;
volatile boolean fourPhase = false;

static uint8_t device_code_buf = 0;
uint8_t saveCount = 0;
int indexCount = 0;

static uint8_t device_code_rx_buf[1];
static uint8_t save_data_rx_buf[1];
static uint8_t data_rx_buf[1];
static uint8_t data_count[1];
uint8_t myCount = 0;


uint8_t buff_01[PROTOCOL_ONE_FIRTS_PHASE_LENGTH];
uint8_t buff_02[PROTOCOL_ONE_SECOND_PHASE_LENGTH];
uint8_t buff_03[PROTOCOL_ONE_THIRD_PHASE_LENGTH];
uint8_t buff_04[PROTOCOL_ONE_FORTH_PHASE_LENGTH];

uint8_t dataBuffer[250][GLUCOSE_DATA_SIZE];

uint16_t startAddress = 0xD200;

int matrixCount  = 0 ;
boolean sendFlag = false;

uint8_t mYears = 0;
uint8_t mMonth = 0;
uint8_t mDay = 0;
uint8_t mHour = 0;
uint8_t mMin = 0;
uint8_t mSecond = 0;
uint8_t glucoseValueL = 0;
uint8_t glucoseValueH = 0;

int updataCount = 0;
void flip() {
  //  digitalWrite(D13, HIGH);
  //  delay(500);
  //  digitalWrite(D13, LOW);
  //  delay(500);
  //  data_count[0] = myCount;
  //  ble.updateCharacteristicValue(characteristic3.getValueAttribute().getHandle(), data_count, 1);
  //  myCount++;
  if (updataCount > saveCount) {
    updataCount = 0;
  } else {
    ble.updateCharacteristicValue(characteristic3.getValueAttribute().getHandle(), dataBuffer[updataCount], GLUCOSE_DATA_SIZE);
    updataCount++;
  }
}

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
    digitalWrite(D13, HIGH);
    Serial.write(0x80);
  }

  if (Handler->handle == characteristic5.getValueAttribute().getHandle()) {
    digitalWrite(D13, LOW);
    Serial.write(serialNumberReadCommand, 7);
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
  Serial.println("m_uart_rx_handle Called");
  //update characteristic data
  ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), rx_buf, rx_buf_num);
  memset(rx_buf, 0x00, 20);
  rx_state = 0;
}

void d_uart_rx_handle() {
  //update characteristic data
  ble.updateCharacteristicValue(characteristic5.getValueAttribute().getHandle(), device_code_rx_buf, 1);
  memset(device_code_rx_buf, 0x00, 1);
}

void c_uart_rx_handle() {
  //update characteristic data
  ble.updateCharacteristicValue(characteristic6.getValueAttribute().getHandle(), save_data_rx_buf, 1);
  memset(save_data_rx_buf, 0x00, 1);
}

void uart_handle(uint32_t id, SerialIrq event) {
  Serial.println("uart event Called");
  Serial.print("rx_buf_nu before--> "); Serial.println(rx_buf_num);
  // Serial rx IRQ
  while (Serial.available()) {
    if (rx_buf_num < 30) {
      rx_buf[rx_buf_num] = Serial.read();
      rx_buf_num++;
    } else {
      Serial.read();
      rx_buf_num = 0;
    }
  }

  Serial.print("rx_buf_nu after--> "); Serial.println(rx_buf_num);
  Serial.print("fisrtPhase --> "); Serial.println(fisrtPhase);
  Serial.print("secondPhase --> "); Serial.println(secondPhase);
  Serial.print("thirdPhase --> "); Serial.println(thirdPhase);
  for (int i = 0; i < rx_buf_num; i++) {
    Serial.write(rx_buf[i]);
  }
  Serial.println("");
  //  if (event == RxIrq) {
  //    if (rx_state == 0) {
  //      rx_state = 1;
  //      timeout.attach(m_uart_rx_handle, 3.0);
  //      rx_buf_num = 0;
  //    }
  //  }
}

void firstPhaseFunction() {
  digitalWrite(D13, HIGH);
  Serial.write(0x80);
  while (Serial.available()) {
    if (rx_buf_num < PROTOCOL_ONE_FIRTS_PHASE_LENGTH) {
      buff_01[rx_buf_num] = Serial.read();
      rx_buf_num++;
    } else {
      Serial.read();
      rx_buf_num = 0;
    }
  }
  for (int i = 0; i < rx_buf_num; i++) {
    Serial.write(buff_01[i]);
  }

  Serial.println("");
  if (rx_buf_num == PROTOCOL_ONE_FIRTS_PHASE_LENGTH) {
    if (buff_01[0] == 0x80 && buff_01[1] == 0x10 && buff_01[2] == 0x20) { // rx_buf[0] == 0x80 && rx_buf[1] == 0x10 && rx_buf[2] == 0x20 프로토콜 1
      protocolOneFlag = true;
      fisrtPhase = true;
      rx_buf_num = 0;
    }
    else if (buff_01[0] == 0x80 && buff_01[1] == 0x1E && buff_01[2] == 0x2E) {
      protocolThreeFlag = true;
      fisrtPhase = true;
      rx_buf_num = 0;
    } else {
      fisrtPhase = true;
      rx_buf_num = 0;
      //      memset(rx_buf, 0x00, 30);
    }
  }
}

void secondPhaseFunction() {
  digitalWrite(D13, LOW);
  Serial.write(serialNumberReadCommand, 7);
  while (Serial.available()) {
    if (rx_buf_num < PROTOCOL_ONE_SECOND_PHASE_LENGTH) {
      buff_02[rx_buf_num] = Serial.read();
      rx_buf_num++;
    } else {
      Serial.read();
      rx_buf_num = 0;
    }
  }
  for (int i = 0; i < rx_buf_num; i++) {
    Serial.write(buff_02[i]);
  }
  Serial.println("");
  if (rx_buf_num == PROTOCOL_ONE_SECOND_PHASE_LENGTH) {

    device_code_rx_buf[0] = (((buff_02[13] & 0x0f) << 4 ) | (buff_02[14] & 0x0f));
    if (device_code_rx_buf[0] == 0x02) { // 멕시마인경우
      secondPhase = true;
      rx_buf_num = 0;
      //      ticker.attach(&flip, 1.0);
    }
  }
}

/**
   세번째 프로토콜 추가
*/
void ThridPhaseFunction() {
  digitalWrite(D13, LOW);
  Serial.write(savaDataCountGetCommand, 7);
  while (Serial.available()) {
    if (rx_buf_num < PROTOCOL_ONE_THIRD_PHASE_LENGTH) {
      buff_03[rx_buf_num] = Serial.read();
      rx_buf_num++;
    } else {
      Serial.read();
      rx_buf_num = 0;
    }
  }
  for (int i = 0; i < rx_buf_num; i++) {
    Serial.write(buff_03[i]);
  }
  Serial.println("");
  if (rx_buf_num == PROTOCOL_ONE_THIRD_PHASE_LENGTH) {

    device_code_rx_buf[0] = (((buff_03[1] & 0x0f) << 4 ) | (buff_03[2] & 0x0f));
    if (device_code_rx_buf[0] != 0x00) {  // 저장된 데이터의 개수가 0개가 아니라면
      saveCount = device_code_rx_buf[0];
      thirdPhase = true;
      rx_buf_num = 0;
      //      memset(rx_buf, 0x00, 30);
      //      ticker.attach(&flip, 1.0);
    } else {
      thirdPhase = false;
      rx_buf_num = 0;
      //      memset(rx_buf, 0x00, 30);
    }
  }
}



void FourthPhaseFunction() {

  uint8_t tmpGetDataCommand[7] = {0x8b, 0x1d, 0x22, 0x10, 0x20, 0x10, 0x28};

  uint8_t topAddress = (startAddress >> 12) & 0x0f;
  uint8_t topAddressDataCommand = (0x10 | topAddress);
  uint8_t top2Address = (startAddress >> 8) & 0x0f;
  uint8_t top2AddressDataCommand =  (0x20 | top2Address);

  uint8_t bottomAddress = (startAddress >> 4) & 0x0f;
  uint8_t bottomAddressDataCommand = (0x10 | bottomAddress);
  uint8_t bottom2Address = (startAddress & 0x000f);
  uint8_t bottom2AddressDataCommand = (0x20 | bottom2Address);

  tmpGetDataCommand[1] = topAddressDataCommand;
  tmpGetDataCommand[2] = top2AddressDataCommand;
  tmpGetDataCommand[3] = bottomAddressDataCommand;
  tmpGetDataCommand[4] = bottom2AddressDataCommand;

  digitalWrite(D13, HIGH);
  Serial.write(tmpGetDataCommand, 7);

  while (Serial.available()) {
    if (rx_buf_num < PROTOCOL_ONE_FORTH_PHASE_LENGTH) {
      buff_04[rx_buf_num] = Serial.read();
      rx_buf_num++;
    } else {
      Serial.read();
      rx_buf_num = 0;
    }
  }

  //  Serial.println("");

  if (rx_buf_num == PROTOCOL_ONE_FORTH_PHASE_LENGTH) { // 24개의 데이터가 들어왔다면
    sendFlag = false;
    if (matrixCount > saveCount) { //모든 데이터 저장 의미
      digitalWrite(D4, HIGH); // 모든 프로세스 완료 인디케이터 
      Serial.print("matrixCount --> "); Serial.println(matrixCount, HEX);
      fourPhase = true;  //
      rx_buf_num = 0;
      ticker.attach(&flip, 1.0);
      //      memset(rx_buf, 0x00, 30);
    } else {
      int cnt = 0;
      uint8_t mYears = (((buff_04[1] & 0x0f) << 4 ) | (buff_04[2] & 0x0f));
      uint8_t mMonth = (((buff_04[4] & 0x0f) << 4 ) | (buff_04[5] & 0x0f));
      uint8_t mDay = (((buff_04[7] & 0x0f) << 4 ) | (buff_04[8] & 0x0f));
      uint8_t mHour = (((buff_04[10] & 0x0f) << 4 ) | (buff_04[11] & 0x0f));
      uint8_t mMin = (((buff_04[13] & 0x0f) << 4 ) | (buff_04[14] & 0x0f));
      uint8_t mSecond = (((buff_04[16] & 0x0f) << 4 ) | (buff_04[17] & 0x0f));
      uint8_t glucoseValueL = (((buff_04[19] & 0x0f) << 4 ) | (buff_04[20] & 0x0f));
      uint8_t glucoseValueH = (((buff_04[22] & 0x0f) << 4 ) | (buff_04[23] & 0x0f));

      Serial.print("startAddress --> "); Serial.println(startAddress, HEX);
      Serial.print("matrixCount --> "); Serial.println(matrixCount, HEX);
      Serial.print("mYears --> "); Serial.println(mYears, HEX);
      Serial.print("mMonth --> "); Serial.println(mMonth, HEX);
      Serial.print("mDay --> "); Serial.println(mDay, HEX);
      Serial.print("mHour --> "); Serial.println(mHour, HEX);
      Serial.print("mMin --> "); Serial.println(mMin, HEX);
      Serial.print("mSecond --> "); Serial.println(mSecond, HEX);
      Serial.print("glucoseValueL --> "); Serial.println(glucoseValueL, HEX);
      Serial.print("glucoseValueH --> "); Serial.println(glucoseValueH, HEX);
      dataBuffer[matrixCount][cnt++] =  mYears;
      dataBuffer[matrixCount][cnt++] =  mMonth;
      dataBuffer[matrixCount][cnt++] =  mDay;
      dataBuffer[matrixCount][cnt++] =  mHour;
      dataBuffer[matrixCount][cnt++] =  mMin;
      dataBuffer[matrixCount][cnt++] =  mSecond;
      dataBuffer[matrixCount][cnt++] =  glucoseValueL;
      dataBuffer[matrixCount][cnt++] =  glucoseValueH;
      cnt = 0;
      for (int i = 0; i < rx_buf_num; i++) {
        Serial.print(i); Serial.println(buff_04[i], HEX);
//        dataBuffer[matrixCount][i] =  buff_04[i];
      }

      matrixCount++;
      startAddress  += 8;
      //      memset(rx_buf, 0x00, 30);
      rx_buf_num = 0;
    }
    //    device_code_rx_buf[0] = (((rx_buf[1] & 0x0f) << 4 ) | (rx_buf[2] & 0x0f));
    //    if (device_code_rx_buf[0] != 0x00) {  // 저장된 데이터의 개수가 0개가 아니라면
    //      saveCount = device_code_rx_buf[0];
    //      fourPhase = true;
    //      rx_buf_num = 0;
    //      memset(rx_buf, 0x00, 30);
    //      ticker.attach(&flip, 1.0);
    //    } else {
    //      fourPhase = false;
    //      rx_buf_num = 0;
    //      memset(rx_buf, 0x00, 30);
    //    }
  } else {

  }

}

void setup() {
  // put your setup code here, to run once
  Serial.begin(9600);
  //  Serial.attach(uart_handle);
  //ticker.attach(&flip, 1);
  pinMode(D4, OUTPUT);
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

  if (!protocolOneFlag && !protocolThreeFlag) {
    if (!fisrtPhase && !secondPhase && !thirdPhase) {
      //      ble.startAdvertising();
      firstPhaseFunction();
    }
  }

  if (protocolOneFlag && !protocolThreeFlag) { // 만약 에코 메세지에서 들어온 플레그 값이 1번 프로토콜 이라면
    if (fisrtPhase && !secondPhase && !thirdPhase) { // 두번째 페이즈 진행
      secondPhaseFunction();
      Serial.print("rx_buf_nu after--> "); Serial.println(rx_buf_num);
      Serial.print("fisrtPhase --> "); Serial.println(fisrtPhase);
      Serial.print("secondPhase --> "); Serial.println(secondPhase);
      Serial.print("thirdPhase --> "); Serial.println(thirdPhase);
    }

    else if (fisrtPhase && secondPhase && !thirdPhase) {
      ThridPhaseFunction();
      Serial.print("rx_buf_nu after--> "); Serial.println(rx_buf_num);
      Serial.print("fisrtPhase --> "); Serial.println(fisrtPhase);
      Serial.print("secondPhase --> "); Serial.println(secondPhase);
      Serial.print("thirdPhase --> "); Serial.println(thirdPhase);
    }

    else if (fisrtPhase && secondPhase && thirdPhase && !fourPhase) { // 모든 조건을 충족했다면 데이터 읽기 과정 처리
      FourthPhaseFunction();
      Serial.print("saveCount --> "); Serial.println(saveCount);
      Serial.print("fourPhase --> "); Serial.println(fourPhase);
    }

    else if (fisrtPhase && secondPhase && thirdPhase && fourPhase) { // 모든 조건을 충족했다면 데이터 읽기 과정 처리
      if (saveCount != 0) {
        //        for (int i = 0; i < saveCount; i++) {
        //          Serial.println("");
        //          for (int k = 0; k < 24; k++) {
        //            Serial.write(dataBuffer[i][k]);
        //          }
        //          Serial.println("");
        //
        //        }
        //        Serial.println("Data Pharse Done --> ");
        //        Serial.print("mMonth --> "); Serial.println(mMonth, HEX);
        //        Serial.print("mDay --> "); Serial.println(mDay, HEX);
        //        Serial.print("mHour --> "); Serial.println(mHour, HEX);
        //        Serial.print("mMin --> "); Serial.println(mMin, HEX);
        //        Serial.print("mSecond --> "); Serial.println(mSecond, HEX);
        //        Serial.print("glucoseValueL --> "); Serial.println(glucoseValueL, HEX);
        //        Serial.print("glucoseValueH --> "); Serial.println(glucoseValueH, HEX);
        
        ble.waitForEvent();

      }
      //      delay(1000);
    }

  }

  // put your main code here, to run repeatedly:
}
