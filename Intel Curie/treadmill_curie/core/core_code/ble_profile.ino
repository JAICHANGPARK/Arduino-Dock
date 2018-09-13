/**
 * 
 * Bluetooth Low Energy Profile 
 * Made by Dreamwalker 
 * for Treadmill
 */

BLEService heartRateService("180D"); // BLE Heart rates Service
BLECharacteristic heartRateMeansurement("2A37", BLERead | BLENotify , 2);
BLEService fitnessMachineService("1826"); // BLE FitnessMachine Service
BLECharacteristic indorBikeChar("2AD2", BLENotify, 4);
BLECharacteristic treadmillChar("2ACD", BLENotify, 5);

BLEService dateTimeService("DDD0");
BLECharacteristic dateTimeSyncChar("0001", BLERead | BLEWrite , 20);
BLECharacteristic resultChar("fff0", BLERead | BLEWrite | BLENotify, 4);

BLEService userAuthService("EEE0");
BLECharacteristic authChar("EEE1", BLERead | BLEWrite , 16);

BLEService dataSyncService("FFF0");
BLECharacteristic controlChar("FFF1", BLERead | BLEWrite , 10);
BLECharacteristic syncChar("FFF2", BLERead | BLEWrite | BLENotify , 20);

void bleProfileSetUp() {
  BLE.setAdvertisedService(heartRateService);
  heartRateService.addCharacteristic(heartRateMeansurement);

  BLE.setAdvertisedService(fitnessMachineService);
  fitnessMachineService.addCharacteristic(indorBikeChar);
  fitnessMachineService.addCharacteristic(treadmillChar);

  BLE.setAdvertisedService(dateTimeService);
  dateTimeService.addCharacteristic(dateTimeSyncChar);
  dateTimeService.addCharacteristic(resultChar);

  BLE.setAdvertisedService(userAuthService);
  userAuthService.addCharacteristic(authChar);

  BLE.setAdvertisedService(dataSyncService);
  dataSyncService.addCharacteristic(controlChar);
  dataSyncService.addCharacteristic(syncChar);

  BLE.addService(heartRateService);   // Add the BLE Heart Rate service
  BLE.addService(fitnessMachineService);   // Add the BLE Indore Bike service
  BLE.addService(dateTimeService);  // 시간 동기화를 위한 서비스 추가
  BLE.addService(userAuthService);  // 사용자 인증을 위한 서비스 추가
  BLE.addService(dataSyncService);  // 데이터 동기화를 위한 서비스 추가

}

void setupBluetoothLowEnergy() {
  //저전력 블루투스 설정
  BLE.begin();
  BLE.setLocalName("KNU TM0"); // 트레드밀 이름 : KNU TM0

  bleProfileSetUp();
  heartRateMeansurement.setValue(heartRateData, 2 );   // initial value for this characteristic
  indorBikeChar.setValue(indoorBikeData, 4);
  treadmillChar.setValue(treadmillData, 5);
  uint8_t initResultPacket[3] = {0,};
  initResultPacket[0] = 0x00;
  initResultPacket[1] = 0x00;
  initResultPacket[2] = 0x00;
  resultChar.setValue(initResultPacket, 3);

  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  dateTimeSyncChar.setEventHandler(BLEWritten, datatimeSyncCharacteristicWritten);
  authChar.setEventHandler(BLEWritten, authCharCharacteristicWritten);
  controlChar.setEventHandler(BLEWritten, dataSyncCharCharacteristicWritten);
  syncChar.setEventHandler(BLEWritten, syncCharCharacteristicWritten);
  BLE.advertise(); // start advertising
}

/******************************************************************************
                       Bluetooth Callback Function
                       @author: 박제창
******************************************************************************/

/**
   블루투스 연결됬을때 발생하는 콜백 함수 (=인터럽트서비스루틴)
   1. 블루투스 연결 플래그 ON
   @author: 박제창
*/
void blePeripheralConnectHandler(BLEDevice central) {
  digitalWrite(BLE_LED_INDICATOR_PIN, HIGH);
  deviceConnectedFlag = true;    // 블루투스 연결 플레그 ONs

#ifdef DEBUG
  Serial.print("Connected to central: ");
  Serial.println(central.address()); // print the central's MAC address:
#endif

}

/**
   블루투스 연결이 종료되면 발생하는 콜백함수
   1. 블루투스 연결 플래그 OFF
   2. 블루투스 결과 전송 패킷 초기화
   3. 디버깅 : 만약 메크로가 정의 되어 있지 않다면 동작하지 않는다.

   @author: 박제창
*/
void blePeripheralDisconnectHandler(BLEDevice central) {

  digitalWrite(BLE_LED_INDICATOR_PIN, LOW);
  deviceConnectedFlag = false;      // 블루투스 연결 플레그 OFF

  // 연결 종료후 패킷 초기화
  resultPacket[0] = 0x00;
  resultPacket[1] = 0x00;
  resultPacket[2] = 0x00;
  resultChar.setValue(resultPacket, 3);

#ifdef DEBUG
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
#endif
}

/*
   날짜 시간 동기화 콜백 함수
   이동 단말로부터 날짜/ 시간 정보를 전달 받아 프로세서 내부 RTC에 설정한다.
    @author: 박제창
*/
void datatimeSyncCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED
  size_t len = dateTimeSyncChar.valueLength();
#ifdef DEBUG
  Serial.println("Characteristic event, written: datatimeSyncCharacteristicWritten ");
  Serial.print("len --> "); Serial.println(len);
#endif
  const unsigned char *data = dateTimeSyncChar.value();  // 들어온 데이터를 포인트 배열에 넣는 핵심 코드
#ifdef DEBUG
  for (size_t i = 0; i < len; i++) {
    Serial.println(data[i], HEX);
  }
#endif

  if (len != 0 && len == 7 ) {
    int receiveYear = ((data[0] << 8 ) & 0xff00 | (data[1] & 0xff));
    int receiveMonth = (data[2] & 0xff);
    int receiveDay = (data[3] & 0xff);
    int receiveHour = (data[4] & 0xff);
    int receiveMinute = (data[5] & 0xff);
    int receiveSecond = (data[6] & 0xff);
    // 날짜/ 시간 설정 함수
    setTime(receiveHour, receiveMinute, receiveSecond, receiveDay, receiveMonth, receiveYear); // 시, 분, 초 ,일, 월, 년
    resultPacket[0] = 0x02;
    resultPacket[1] = 0x01;
    resultPacket[2] = 0x03;
    resultChar.setValue(resultPacket, 3);
    bleDateTimeSycnFlag = true;     // 성공시 날짜 시간 동기화 플래그 ON

#ifdef DEBUG
    Serial.print("날짜 값 변환 년 --> "); Serial.println(receiveYear);
    Serial.print("날짜 값 변환 월--> "); Serial.println(receiveMonth);
    Serial.print("날짜 값 변환 일--> "); Serial.println(receiveDay);
    Serial.print("날짜 값 변환 시--> "); Serial.println(receiveHour);
    Serial.print("날짜 값 변환 분--> "); Serial.println(receiveMinute);
    Serial.print("날짜 값 변환 초--> "); Serial.println(receiveSecond);
#endif

  } else {
    //날짜 정보가 올바르지 않은 경우
    resultPacket[0] = 0x02;
    resultPacket[1] = 0xff;
    resultPacket[2] = 0x03;
    resultChar.setValue(resultPacket, 3);
    bleDateTimeSycnFlag = false;    // 실패시 날짜 시간 동기화 플래그 OFF
  }
}

/**
   디바이스 인증 과정 처리하는 콜백 함수
   모바일 애플리케이션에서 암호화된 16바이트의 정보를 받아
   복호화 한 후 검증한다.
   1. 특성 데이터 읽기 --> 버퍼 저장
   2. 복호화
   3. 검증
   4. 결과 패킷 전송

   @author: 박제창
*/
void authCharCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED

  size_t len = authChar.valueLength();
#ifdef DEBUG
  Serial.println("Characteristic event, written: authCharCharacteristicWritten ");
  Serial.print("len --> "); Serial.println(len);
#endif
  const unsigned char *data = authChar.value();
  byte receiveData[16];
  byte aes_result[16];

  for (int i = 0; i < 16; i++) {
    receiveData[i] = data[i];
  }
#ifdef DEBUG
  for (size_t i = 0; i < len; i++) {
    Serial.print(data[i], HEX);
    Serial.print(",");
  }
#endif
  AES.Decrypt(receiveData, 16, aes_result, 0);
#ifdef DEBUG
  Serial.println("Decrypt");
  for (int i = 0; i < 16; i++) {
    Serial.print(aes_result[i], HEX);
    Serial.print(",");
  }
#endif

  boolean authFlag = false;   // 검증 확인용 내부 스위치 플래그  일치시 : True, 불 일치시 : False

  for (int i = 0; i < 16; i++) {
    if (!authCoreValue[i] == aes_result[i]) {
      //      Serial.println("Error");
      authFlag = false;

    } else {
      //      Serial.print("SUCESS");
      authFlag = true;
    }
  }
  if (authFlag) {
    authData[0] = 0x02;
    authData[1] = 0x02;
    authData[2] = 0x03;
    resultChar.setValue(authData, 3); //resultChar 담아 변수 설정.
    bleAuthCheckFlag = true;    // 성공시 디바이스 인증 플래그 ON
  } else {
    authData[0] = 0x02;
    authData[1] = 0xFF;
    authData[2] = 0x03;
    resultChar.setValue(authData, 3); //resultChar 담아 변수 설정.
    bleAuthCheckFlag = false;   // 실패시 디바이스 인증 플래그 OFF
  }

}

/**
    데이터 동기화를 위한 콜백함수 - 박제창
    데이터 전송 요청
    0x00 : 전부
    0xyy : 아직 정해지지 않음 Reserved
    boolean bleDateTimeSycnFlag = false // 블루투스를 통해 시간 동기화가 되었을시 처리하는 플래그
    boolean bleAuthCheckFlag = false // 사용자 인증을 위한 플래그 실패시 false 성공시 true
    boolean bleDataSyncFlag = false // 데이터 전송 요청 이 들어왔을겨우 올바른 데이터 형식이면 true 아니면 false

    @author: 박제창

*/
void dataSyncCharCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED

  size_t len = controlChar.valueLength();
#ifdef DEBUG
  Serial.println("Characteristic event, written: dataSyncCharCharacteristicWritten ");
  Serial.print("len --> "); Serial.println(len);
#endif
  const unsigned char *data = controlChar.value();

  // 동기화를 위해 시간 동기화, 인증 동기화가 모두 완벽하게 진행됬다면
  if (bleDateTimeSycnFlag && bleAuthCheckFlag) {
#ifdef DEBUG
    Serial.println("enter success flag  ");
    Serial.print("len --> "); Serial.println(len);
    for (size_t i = 0; i < len; i++) {
      Serial.println(data[i], HEX);
    }
#endif
    if (len >= 3) { // 사전 동기화 과정이 완료되고 길이가 3개인패킷이 들어왔다면
      if (data[0] == 0x02 && data[2] == 0x03) { // 시작 신호와 종료신호가 올바르다면
#ifdef DEBUG
        Serial.println("시작신호 종료신호 잘 들어옴 ");
#endif
        if (data[1] == 0x00) {  // 중간의 명령어가 올바르다면 0x00 : 모두 전송
#ifdef DEBUG
          Serial.print("register_index --> "); Serial.println(register_index);
          Serial.println("모든 레지스터 데이터 전송");
          Serial.print("전송 플래그 설정 완료  선공");
#endif
          bleDataSyncFlag = true;    // 데이터 동기화 패킷이 올바르게 들어왔다면 동기화 플래그 ON
          // 전송 처리는 LOOP statement에서 처리해준다.
          // 왜냐하면 Stuck 현상이 발생하기 때문에
        }
        //        // 전송 성공 시
        //        resultPacket[0] = 0x02;
        //        resultPacket[1] = 0x03;
        //        resultPacket[2] = 0x03;
        //        resultChar.setValue(resultPacket, 3);

      } else { // 시작 신호(0x02)와 종료신호(0x03)가 일치 하지 않다면
        bleDataSyncFlag = false;
        resultPacket[0] = 0x02;
        resultPacket[1] = 0xff;
        resultPacket[2] = 0x03;
        resultChar.setValue(resultPacket, 3);
      }
    } else { // 길이가 3패킷보다 작다면
      bleDataSyncFlag = false;
      resultPacket[0] = 0x02;
      resultPacket[1] = 0xff;
      resultPacket[2] = 0x03;
      resultChar.setValue(resultPacket, 3);
    }
  } else {
    // 이전 단계의 모든 인증 실패 시
    bleDataSyncFlag = false;
    resultPacket[0] = 0x02;
    resultPacket[1] = 0xff;
    resultPacket[2] = 0x03;
    resultChar.setValue(resultPacket, 3);
  }
}

void syncCharCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {

}


